#include "level_curve.h"
#include "map_placement.h"
#include "drawn_iterates.h"
#include "surface_levels.h"
#include "surface_figures.h"
#include "surface_overlay.h"
#include "scenario_defaults.h"

#include <threepp/math/Color.hpp>

#include <threepp/materials/MeshPhongMaterial.hpp>

#include <array>
#include <cmath>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <iterator>
#include <optional>
#include <algorithm>

namespace praxis::optimization {

namespace {

std::shared_ptr<threepp::Material> toned(std::uint32_t tone)
{
    auto material   = threepp::MeshPhongMaterial::create();
    material->color = threepp::Color(tone);

    return material;
}

void drawn_at(threepp::Material &material, double opacity)
{
    material.transparent = opacity < 1.0;
    material.opacity     = static_cast<float>(opacity);
}

}

std::vector<double> map_levels(const scenario_grid &grid)
{
    std::vector<double> sorted;
    std::ranges::copy_if(grid.heights, std::back_inserter(sorted), [](double f) { return std::isfinite(f); });
    std::ranges::sort(sorted);
    std::vector<double> levels;
    if(sorted.size() < 2u || !(sorted.front() < sorted.back()))
        return levels;

    for(std::size_t k = 1; k <= level_count; ++k)
    {
        const double at         = static_cast<double>(k) * static_cast<double>(sorted.size() - 1u) / static_cast<double>(level_count + 1u);
        const auto below        = static_cast<std::size_t>(at);
        const std::size_t above = std::min(below + 1u, sorted.size() - 1u);
        levels.push_back(sorted[below] + (at - static_cast<double>(below)) * (sorted[above] - sorted[below]));
    }

    return levels;
}

level_map::level_map(std::shared_ptr<threepp::BufferGeometry> sphere)
        : m_pointed(false)
        , m_stepped(false)
        , m_floored(false)
        , m_floor(0.0)
        , m_arrow(arrow_object("gradient at x_k", gradient_tone))
        , m_map(threepp::Group::create())
        , m_step(rod_object("step taken", floor_step_tone, floor_step_radius))
        , m_curves(occluded_lines("level curves", level_curve_tone, level_count * level_curve_room(surface_samples), false))
        , m_projected(threepp::Group::create())
        , m_mark_tone(toned(0xffffff))
        , m_sphere(std::move(sphere))
        , m_marks()
        , m_sampled()
        , m_placed()
{
    m_map->name       = "level map";
    m_map->visible    = false;
    m_projected->name = "iterates on the floor";
    for(const std::shared_ptr<threepp::Object3D> &part : std::array<std::shared_ptr<threepp::Object3D>, 4>{m_curves, m_projected, m_arrow.object, m_step})
        m_map->add(part);
}

const std::shared_ptr<threepp::Group> &level_map::object() const
{
    return m_map;
}

void level_map::draw(const std::shared_ptr<const scenario_snapshot> &shown, const field_placement &placed, const scenario_view &view)
{
    const bool moved = view.floor_projection != m_floored;
    m_floored        = view.floor_projection;
    if(shown->grid != m_sampled || moved)
        resample(*shown, placed);
    if(shown != m_placed || moved)
        place(*shown, placed);
    m_placed = shown;
}

void level_map::show(const scenario_view &view)
{
    m_curves->visible       = view.level_curves && m_curves->geometry()->drawRange.count > 0;
    m_projected->visible    = view.iterates && view.floor_projection;
    m_arrow.object->visible = view.gradient && m_pointed;
    m_step->visible         = view.step && m_stepped;
}

void level_map::resample(const scenario_snapshot &shown, const field_placement &placed)
{
    m_floor                = floor_beneath(*shown.grid, placed);
    const map_placement at = standing(placed);
    std::vector<Eigen::Vector3d> ends;
    for(const double level : map_levels(*shown.grid))
    {
        const std::vector<Eigen::Vector3d> traced = ends_at(level_curve(*shown.grid, shown.ranges, level), level_height(at, level));
        ends.insert(ends.end(), traced.begin(), traced.end());
    }
    write_points(*m_curves, ends, in_meters(placed));
    m_sampled = shown.grid;
}

void level_map::place(const scenario_snapshot &shown, const field_placement &placed)
{
    std::vector<Eigen::Vector3d> centers;
    std::vector<std::uint32_t> tones;
    for(const drawn_iterate &mark : drawn_iterates(shown, placed, iterate_spacing))
    {
        centers.emplace_back(mark.at.x(), mark.at.y(), m_floor);
        tones.push_back(mark.iterate == shown.step ? floor_selected_iterate_tone : floor_iterate_tone);
    }
    hold_marks(*m_projected, m_marks, centers.size(), marks_look{"iterate on the floor", floor_iterate_tone, m_sphere, m_mark_tone});
    if(m_marks != nullptr)
        write_marks(*m_marks, centers, tones);
    step_from(shown, standing(placed));
}

void level_map::step_from(const scenario_snapshot &shown, const map_placement &at)
{
    const std::size_t k                       = shown.step;
    const std::optional<Eigen::Vector2d> g    = answered_gradient(shown, k);
    const std::optional<Eigen::Vector3d> from = standing_at(shown, k, at);
    m_pointed                                 = g && from && g->norm() > 0.0;
    m_stepped                                 = false;
    lay_over_field(*m_arrow.object, !at.floor);
    lay_over_field(*m_step, !at.floor);
    if(!g || !from)
        return;

    if(m_pointed)
        place_arrow(m_arrow, *from, Eigen::Vector3d(g->x(), g->y(), 0.0), gradient_arrow_fraction * surface_extent);
    if(const std::optional<std::array<Eigen::Vector3d, 2>> ends = step_ends(shown, k, at))
    {
        place_rod(*m_step, (*ends)[0], (*ends)[1]);
        m_stepped = (*ends)[0] != (*ends)[1];
    }
}

map_placement level_map::standing(const field_placement &placed) const
{
    return map_placement{placed, m_floored ? std::optional<double>(m_floor) : std::nullopt};
}

void draw_map(level_map &map, const form_materials &drawn, const std::shared_ptr<const scenario_snapshot> &shown, const field_placement &placed, const scenario_view &view)
{
    const bool minimized = shown->run.minimized_by.has_value();
    drawn_at(drawn.field, minimized ? minimum_field_opacity : 1.0);
    drawn_at(drawn.plane, minimized ? minimum_plane_opacity : 1.0);
    drawn_at(drawn.central, minimized ? minimum_plane_opacity : 1.0);
    map.object()->visible = minimized;
    if(!minimized)
        return;

    map.draw(shown, placed, view);
    map.show(view);
}

}
