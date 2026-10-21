#include "level_curve.h"
#include "surface_mesh.h"
#include "drawn_iterates.h"
#include "surface_figures.h"
#include "surface_overlay.h"
#include "scenario_defaults.h"

#include "praxis/optimization/surface_stencil.h"

#include <threepp/math/Color.hpp>
#include <threepp/math/Matrix4.hpp>
#include <threepp/math/MathUtils.hpp>

#include <threepp/materials/MeshPhongMaterial.hpp>

#include <threepp/geometries/SphereGeometry.hpp>

#include <Eigen/Core>

#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
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

// Two segments at most cross each cell, two points each.
constexpr std::size_t zero_curve_room = 4u * (surface_samples - 1u) * (surface_samples - 1u);

// Sutherland-Hodgman leaves at most eight corners of a square held to a rectangle, and six of a
// rectangle held between two parallel lines.
constexpr std::size_t patch_corners = 8u;
constexpr std::size_t plane_corners = 6u;

void hang(threepp::Object3D &under, std::initializer_list<std::shared_ptr<threepp::Object3D>> figures)
{
    for(const std::shared_ptr<threepp::Object3D> &figure : figures)
        under.add(figure);
}

// The first `drawn` instances are uploaded on the next frame and culled by where they now stand.
void show_written(threepp::InstancedMesh &marks, std::size_t drawn)
{
    marks.setCount(drawn);
    marks.instanceMatrix()->needsUpdate();
    marks.instanceColor()->needsUpdate();
    marks.computeBoundingSphere();
}

std::vector<Eigen::Vector3d> on_zero_plane(const std::vector<level_segment> &segments)
{
    std::vector<Eigen::Vector3d> points;
    for(const level_segment &segment : segments)
        for(const Eigen::Vector2d &end : segment)
            points.emplace_back(end.x(), end.y(), 0.0);

    return points;
}

}

// The scene's frame is y-up and the surface is written z-up, so everything hangs under one root
// carrying that quarter turn.
surface_stencil::surface_stencil(threepp::Scene &parent, scenario_reader seen, std::shared_ptr<const surface_view> drawn)
        : m_scene(parent)
        , m_seen(std::move(seen))
        , m_view(std::move(drawn))
        , m_root(threepp::Group::create())
        , m_field(threepp::Mesh::create(field_geometry(surface_samples), lit(surface_tone, true)))
        , m_iterates(threepp::Group::create())
        , m_iterate_material(lit(0xffffff, false))
        , m_sphere(threepp::SphereGeometry::create(static_cast<float>(iterate_radius)))
        , m_zero_curve(overlay_lines("zero curve", zero_curve_tone, zero_curve_room, false))
        , m_box(overlay_lines("box", box_tone, 8u, false))
        , m_step(threepp::Group::create())
        , m_patch(plane_patch("tangent plane", tangent_plane_tone, patch_corners, false))
        , m_zero_line(overlay_lines("zero line", zero_line_tone, 2u, true))
        , m_nearest(overlay_point("nearest point", zero_line_tone, m_sphere))
        , m_recorded(overlay_lines("x_(k+1)", iterate_tone, 2u, true))
        , m_plane(plane_patch("tangent plane at p", tangent_plane_tone, plane_corners, false))
        , m_central(plane_patch("central difference at p", tangent_plane_tone, plane_corners, true))
        , m_sampled()
        , m_placed()
        , m_heights{0.0, 0.0}
        , m_iterate_marks()
{
    m_root->rotation.x = -threepp::math::PI / 2.f;
    m_field->name      = "surface";
    m_iterates->name   = "iterates";
    m_step->name       = "step";
    hang(*m_step, {m_patch, m_zero_line, m_nearest, m_recorded});
    hang(*m_root, {m_field, m_iterates, m_zero_curve, m_box, m_step, m_plane, m_central});
}

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
    const surface_view shown_parts = m_view != nullptr ? *m_view : surface_view{};
    m_field->visible               = shown_parts.field;
    m_iterates->visible            = shown_parts.iterates;
    m_step->visible                = shown_parts.step;

    const std::shared_ptr<const scenario_snapshot> shown = m_seen.read();
    if(shown == nullptr || shown->grid == nullptr || shown->ranges.size() != 2u)
        return;

    const field_placement placed = placement_of(shown->ranges, *shown->grid);
    if(shown->grid != m_sampled)
        resample(*shown);
    m_zero_curve->visible = shown_parts.zero_curve && m_zero_curve->geometry()->drawRange.count > 0;
    write_points(*m_box, box_outline(shown->bounds, shown->ranges), placed);
    m_box->visible = m_box->visible && shown_parts.bounds;
    place_iterates(shown);
    draw_step(step_figures{*m_patch, *m_zero_line, *m_nearest, *m_recorded}, *shown, placed);
    draw_planes(point_figures{*m_plane, *m_central}, *shown, placed, m_heights);
    m_plane->visible   = m_plane->visible && shown_parts.point;
    m_central->visible = m_central->visible && shown_parts.point;
}

// The field, its zero curve and the heights the planes are held to change only with the samples.
void surface_stencil::resample(const scenario_snapshot &shown) const
{
    const field_placement placed = placement_of(shown.ranges, *shown.grid);
    write_field(*m_field->geometry(), *shown.grid, shown.ranges, placed);
    write_points(*m_zero_curve, on_zero_plane(level_curve(*shown.grid, shown.ranges, 0.0)), placed);
    m_heights = height_span(*shown.grid);
    m_sampled = shown.grid;
}

void surface_stencil::place_iterates(const std::shared_ptr<const scenario_snapshot> &shown) const
{
    if(shown == m_placed)
        return;

    m_placed                               = shown;
    const std::vector<drawn_iterate> drawn = drawn_iterates(*shown, placement_of(shown->ranges, *shown->grid), iterate_spacing);
    hold_marks(drawn.size());
    if(m_iterate_marks == nullptr)
        return;

    for(std::size_t j = 0; j < drawn.size(); ++j)
    {
        const Eigen::Vector3d &at = drawn[j].at;
        m_iterate_marks->setMatrixAt(j, threepp::Matrix4().makeTranslation(static_cast<float>(at.x()), static_cast<float>(at.y()), static_cast<float>(at.z())));
        m_iterate_marks->setColorAt(j, threepp::Color(drawn[j].iterate == shown->step ? selected_iterate_tone : iterate_tone));
    }
    show_written(*m_iterate_marks, drawn.size());
}

// One instance per drawn iterate, and a mesh of its own for another count.
void surface_stencil::hold_marks(std::size_t count) const
{
    if(m_iterate_marks != nullptr && static_cast<std::size_t>(m_iterate_marks->instanceMatrix()->count()) == count)
        return;

    if(m_iterate_marks != nullptr)
        m_iterates->remove(*m_iterate_marks);
    m_iterate_marks = nullptr;
    if(count == 0u)
        return;

    m_iterate_marks       = threepp::InstancedMesh::create(m_sphere, m_iterate_material, count);
    m_iterate_marks->name = "iterate";
    // threepp picks the shader by whether instance colors exist at the first draw, so they exist from the start.
    m_iterate_marks->setColorAt(0u, threepp::Color(iterate_tone));
    m_iterates->add(m_iterate_marks);
}

}
