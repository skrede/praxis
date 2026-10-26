#include "level_curve.h"
#include "surface_mesh.h"
#include "surface_keyed.h"
#include "drawn_iterates.h"
#include "surface_levels.h"
#include "surface_figures.h"
#include "surface_overlay.h"
#include "scenario_defaults.h"

#include "praxis/optimization/surface_stencil.h"

#include <threepp/math/Color.hpp>
#include <threepp/math/MathUtils.hpp>

#include <threepp/materials/MeshPhongMaterial.hpp>

#include <threepp/geometries/SphereGeometry.hpp>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <algorithm>
#include <initializer_list>

namespace praxis::optimization {

namespace {

std::shared_ptr<threepp::Material> lit(std::uint32_t tone, bool both_sides)
{
    auto material   = threepp::MeshPhongMaterial::create();
    material->color = threepp::Color(tone);
    if(both_sides)
        material->side = threepp::Side::Double;

    return material;
}

// Sutherland-Hodgman leaves at most eight corners of a square held to a rectangle, and six of a
// rectangle held between two parallel lines.
constexpr std::size_t patch_corners = 8u;
constexpr std::size_t plane_corners = 6u;

void hang(threepp::Object3D &under, std::initializer_list<std::shared_ptr<threepp::Object3D>> figures)
{
    for(const std::shared_ptr<threepp::Object3D> &figure : figures)
        under.add(figure);
}

}

// The scene's frame is y-up and the surface is written z-up, so everything hangs under one root
// carrying that quarter turn.
surface_stencil::surface_stencil(threepp::Scene &parent, scenario_reader seen, std::shared_ptr<const scenario_view> drawn, std::shared_ptr<std::vector<scene::key_entry>> keyed)
        : m_scene(parent)
        , m_seen(std::move(seen))
        , m_view(std::move(drawn))
        , m_keyed(std::move(keyed))
        , m_root(threepp::Group::create())
        , m_field(threepp::Mesh::create(field_geometry(surface_samples), lit(surface_tone, true)))
        , m_iterates(threepp::Group::create())
        , m_iterate_material(lit(0xffffff, false))
        , m_sphere(threepp::SphereGeometry::create(static_cast<float>(iterate_radius)))
        , m_zero_curve(overlay_lines("zero curve", zero_curve_tone, level_curve_room(surface_samples), false))
        , m_box(overlay_lines("box", box_tone, 8u, false))
        , m_step(threepp::Group::create())
        , m_patch(plane_patch("tangent plane", tangent_tone, patch_corners, false))
        , m_zero_line(overlay_lines("zero line", tangent_zero_tone, 2u, true))
        , m_nearest(overlay_point("nearest point", tangent_zero_tone, m_sphere))
        , m_recorded(overlay_lines("x_(k+1)", next_iterate_line_tone, 2u, true))
        , m_plane(plane_patch("tangent plane at p", linearization_point_tone, plane_corners, false))
        , m_central(plane_patch("central difference at p", central_difference_tone, plane_corners, true))
        , m_levels(std::make_unique<level_map>(m_sphere))
        , m_sampled()
        , m_placed()
        , m_heights{0.0, 0.0}
        , m_iterate_marks()
        , m_marked()
{
    m_root->rotation.x = -threepp::math::PI / 2.f;
    m_field->name      = "surface";
    m_iterates->name   = "iterates";
    m_step->name       = "step";
    hang(*m_step, {m_patch, m_zero_line, m_nearest, m_recorded});
    hang(*m_root, {m_field, m_iterates, m_zero_curve, m_box, m_step, m_plane, m_central, m_levels->object()});
}

surface_stencil::~surface_stencil() = default;

expected<void, refusal> surface_stencil::initialize()
{
    m_scene.add(m_root);

    return {};
}

void surface_stencil::tear_down()
{
    m_scene.remove(*m_root);
}

void surface_stencil::render() const
{
    draw();
    if(m_keyed != nullptr)
        write_key();
}

void surface_stencil::draw() const
{
    const scenario_view shown_parts = m_view != nullptr ? *m_view : scenario_view{};
    m_field->visible                = shown_parts.field;
    m_iterates->visible             = shown_parts.iterates;

    const std::shared_ptr<const scenario_snapshot> shown = m_seen.read();
    if(shown == nullptr || shown->grid == nullptr || shown->ranges.size() != 2u)
        return;

    const bool minimized         = shown->run.minimized_by.has_value();
    const field_placement placed = placement_of(shown->ranges, *shown->grid);
    if(shown->grid != m_sampled)
        resample(*shown);
    m_zero_curve->visible = shown_parts.zero_curve && !minimized && m_zero_curve->geometry()->drawRange.count > 0;
    m_step->visible       = shown_parts.step && !minimized;
    draw_map(*m_levels, form_materials{*m_field->material(), *m_plane->material(), *m_central->material()}, shown, placed, shown_parts);
    write_points(*m_box, box_outline(shown->bounds, shown->ranges), placed);
    m_box->visible = m_box->visible && shown_parts.bounds;
    place_iterates(shown);
    draw_step(step_figures{*m_patch, *m_zero_line, *m_nearest, *m_recorded}, *shown, placed);
    draw_planes(point_figures{*m_plane, *m_central}, *shown, placed, m_heights);
    m_plane->visible   = m_plane->visible && shown_parts.linearization;
    m_central->visible = m_central->visible && shown_parts.linearization;
}

// The field, its zero curve and the heights the planes are held to change only with the samples.
void surface_stencil::resample(const scenario_snapshot &shown) const
{
    const field_placement placed = placement_of(shown.ranges, *shown.grid);
    write_field(*m_field->geometry(), *shown.grid, shown.ranges, placed);
    write_points(*m_zero_curve, ends_at(level_curve(*shown.grid, shown.ranges, 0.0), 0.0), placed);
    m_heights = height_span(*shown.grid);
    m_sampled = shown.grid;
}

void surface_stencil::place_iterates(const std::shared_ptr<const scenario_snapshot> &shown) const
{
    if(shown == m_placed)
        return;

    m_placed                               = shown;
    const std::vector<drawn_iterate> drawn = drawn_iterates(*shown, placement_of(shown->ranges, *shown->grid), iterate_spacing);
    hold_marks(*m_iterates, m_iterate_marks, drawn.size(), marks_look{"iterate", iterate_tone, m_sphere, m_iterate_material});
    std::vector<Eigen::Vector3d> centers;
    m_marked.clear();
    for(const drawn_iterate &mark : drawn)
    {
        centers.push_back(mark.at);
        m_marked.push_back(mark.iterate == shown->step ? selected_iterate_tone : iterate_tone);
    }
    if(m_iterate_marks != nullptr)
        write_marks(*m_iterate_marks, centers, m_marked);
}

// A floor marker stands for each iterate drawn on the field, so the field's tones say which are drawn.
void surface_stencil::write_key() const
{
    threepp::Object3D &map = *m_levels->object();
    const auto marked      = [this](std::uint32_t tone) { return std::ranges::find(m_marked, tone) != m_marked.end(); };
    const bool on_field    = shown_through(*m_iterates);
    const bool on_floor    = shown_through(*map.getObjectByName("iterates on the floor"));
    const std::array<keyed_figure, 16> figures{keyed_figure{shown_through(*m_field) && m_field->geometry()->drawRange.count > 0, "f(x, y)", surface_tone},
                                               keyed_figure{on_field && marked(iterate_tone), "Iterates", iterate_tone},
                                               keyed_figure{on_field && marked(selected_iterate_tone), "x_k", selected_iterate_tone},
                                               keyed_figure{shown_through(*m_zero_curve), "zero curve", zero_curve_tone},
                                               keyed_figure{shown_through(*map.getObjectByName("level curves")), "level curves", level_curve_tone},
                                               keyed_figure{shown_through(*m_patch), "tangent at x_k", tangent_tone},
                                               keyed_figure{shown_through(*m_zero_line), "zero line", tangent_zero_tone},
                                               keyed_figure{shown_through(*m_nearest), "nearest point", tangent_zero_tone},
                                               keyed_figure{shown_through(*m_recorded), "x_(k+1)", next_iterate_line_tone},
                                               keyed_figure{shown_through(*map.getObjectByName("gradient at x_k")), "gradient at x_k", gradient_tone},
                                               keyed_figure{shown_through(*map.getObjectByName("step taken")), "step taken", floor_step_tone},
                                               keyed_figure{on_floor && marked(iterate_tone), "floor iterates", floor_iterate_tone},
                                               keyed_figure{on_floor && marked(selected_iterate_tone), "x_k on the floor", floor_selected_iterate_tone},
                                               keyed_figure{shown_through(*m_box), "bounds", box_tone},
                                               keyed_figure{shown_through(*m_plane), "tangent at p", linearization_point_tone},
                                               keyed_figure{shown_through(*m_central), "difference at p", central_difference_tone}};
    *m_keyed = drawn_entries(figures);
}

}
