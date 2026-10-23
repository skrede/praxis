#include "level_curve.h"
#include "surface_step.h"
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

// The floor is written in meters, so a point (x, y, z) stands over (x, y) at the height z.
field_placement on_floor(const field_placement &placed)
{
    return field_placement{placed.middle, placed.across, 1.0};
}

void drawn_at(threepp::Material &material, double opacity)
{
    material.transparent = opacity < 1.0;
    material.opacity     = static_cast<float>(opacity);
}

// grad f at x_k where x_k lies within the ranges and the gradient was answered finitely.
std::optional<Eigen::Vector2d> answered_gradient(const scenario_snapshot &shown, std::size_t k)
{
    const scenario_run &run = shown.run;
    if(k >= run.iterates.size() || k >= run.gradients.size() || !within_ranges(shown.ranges, run.iterates[k]))
        return std::nullopt;

    const expected<Eigen::VectorXd, refusal> &g = run.gradients[k];
    if(!g || g->size() != 2 || !g->allFinite())
        return std::nullopt;

    return Eigen::Vector2d((*g)[0], (*g)[1]);
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
        , m_floor(0.0)
        , m_arrow(arrow_object("gradient at x_k", gradient_tone))
        , m_map(threepp::Group::create())
        , m_step(occluded_lines("step on the floor", floor_step_tone, 2u, true))
        , m_curves(occluded_lines("level curves", level_curve_tone, level_count * level_curve_room(surface_samples), false))
        , m_projected(threepp::Group::create())
        , m_mark_tone(toned(0xffffff))
        , m_sphere(std::move(sphere))
        , m_marks()
        , m_sampled()
        , m_placed()
{
    m_map->name       = "level map";
    m_projected->name = "iterates on the floor";
    for(const std::shared_ptr<threepp::Object3D> &part : std::array<std::shared_ptr<threepp::Object3D>, 4>{m_curves, m_projected, m_arrow.object, m_step})
        m_map->add(part);
}

const std::shared_ptr<threepp::Group> &level_map::object() const
{
    return m_map;
}

void level_map::draw(const std::shared_ptr<const scenario_snapshot> &shown, const field_placement &placed)
{
    if(shown->grid != m_sampled)
        resample(*shown, placed);
    if(shown != m_placed)
        place(*shown, placed);
    m_placed = shown;
}

void level_map::show(const scenario_view &view)
{
    m_curves->visible       = view.level_curves && m_curves->geometry()->drawRange.count > 0;
    m_projected->visible    = view.iterates;
    m_arrow.object->visible = view.gradient && m_pointed;
    m_step->visible         = view.step && m_step->geometry()->drawRange.count > 0;
}

void level_map::resample(const scenario_snapshot &shown, const field_placement &placed)
{
    m_floor = placed.vertical * height_span(*shown.grid).lower - floor_gap * surface_extent;
    std::vector<Eigen::Vector3d> ends;
    for(const double level : map_levels(*shown.grid))
    {
        const std::vector<Eigen::Vector3d> traced = ends_at(level_curve(*shown.grid, shown.ranges, level), m_floor);
        ends.insert(ends.end(), traced.begin(), traced.end());
    }
    write_points(*m_curves, ends, on_floor(placed));
    m_sampled = shown.grid;
}

void level_map::place(const scenario_snapshot &shown, const field_placement &placed)
{
    std::vector<Eigen::Vector3d> centers;
    for(const drawn_iterate &mark : drawn_iterates(shown, placed, iterate_spacing))
        centers.emplace_back(mark.at.x(), mark.at.y(), m_floor);
    hold_marks(*m_projected, m_marks, centers.size(), marks_look{"iterate on the floor", floor_iterate_tone, m_sphere, m_mark_tone});
    if(m_marks != nullptr)
        write_marks(*m_marks, centers, std::vector<std::uint32_t>(centers.size(), floor_iterate_tone));
    step_from(shown, on_floor(placed));
}

void level_map::step_from(const scenario_snapshot &shown, const field_placement &floor)
{
    const std::size_t k                    = shown.step;
    const std::optional<Eigen::Vector2d> g = answered_gradient(shown, k);
    m_pointed                              = g && g->norm() > 0.0;
    write_points(*m_step, std::vector<Eigen::Vector3d>{}, floor);
    if(!g)
        return;

    const Eigen::VectorXd &x = shown.run.iterates[k];
    const Eigen::Vector3d from(x[0], x[1], m_floor);
    if(m_pointed)
        place_arrow(m_arrow, floor.at(x[0], x[1], m_floor), Eigen::Vector3d(g->x(), g->y(), 0.0), gradient_arrow_fraction * surface_extent);
    if(k + 1u < shown.run.iterates.size() && within_ranges(shown.ranges, shown.run.iterates[k + 1u]))
    {
        const Eigen::VectorXd &next = shown.run.iterates[k + 1u];
        write_points(*m_step, std::array<Eigen::Vector3d, 2>{from, Eigen::Vector3d(next[0], next[1], m_floor)}, floor);
    }
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

    map.draw(shown, placed);
    map.show(view);
}

}
