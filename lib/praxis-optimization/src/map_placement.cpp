#include "surface_step.h"
#include "map_placement.h"
#include "surface_overlay.h"
#include "scenario_defaults.h"

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace praxis::optimization {

field_placement in_meters(const field_placement &placed)
{
    return field_placement{placed.middle, placed.across, 1.0};
}

double floor_beneath(const scenario_grid &grid, const field_placement &placed)
{
    return placed.vertical * height_span(grid).lower - floor_gap * surface_extent;
}

double level_height(const map_placement &at, double level)
{
    return at.floor.value_or(at.field.vertical * level);
}

std::optional<Eigen::Vector3d> standing_at(const scenario_snapshot &shown, std::size_t k, const map_placement &at)
{
    const scenario_run &run = shown.run;
    if(k >= run.iterates.size() || !within_ranges(shown.ranges, run.iterates[k]))
        return std::nullopt;

    const Eigen::VectorXd &x = run.iterates[k];
    if(at.floor)
        return in_meters(at.field).at(x[0], x[1], *at.floor);
    if(k >= run.errors.size() || !run.errors[k] || run.errors[k]->size() != 1 || !std::isfinite((*run.errors[k])[0]))
        return std::nullopt;

    return at.field.at(x[0], x[1], -(*run.errors[k])[0]);
}

std::optional<std::array<Eigen::Vector3d, 2>> step_ends(const scenario_snapshot &shown, std::size_t k, const map_placement &at)
{
    const std::optional<Eigen::Vector3d> from = standing_at(shown, k, at);
    const std::optional<Eigen::Vector3d> to   = standing_at(shown, k + 1u, at);
    if(!from || !to)
        return std::nullopt;

    const Eigen::Vector3d raised(0.0, 0.0, at.floor ? floor_step_lift : 0.0);

    return std::array<Eigen::Vector3d, 2>{*from + raised, *to + raised};
}

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
