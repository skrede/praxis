#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_MAP_PLACEMENT_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_MAP_PLACEMENT_H

#include "surface_mesh.h"

#include "praxis/optimization/scenario_snapshot.h"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <optional>

namespace praxis::optimization {

// Where a minimized surface's level map stands in the field's frame: on the field, each level curve
// at its own height and x_k at the center of its marker there, or with a `floor` height, in meters,
// on that floor, both projected straight down onto it.
struct map_placement
{
    field_placement field;
    std::optional<double> floor;
};

// A point (x, y, z) written through it stands over (x, y) at z meters.
field_placement in_meters(const field_placement &placed);

// `floor_gap` of the extent below the field's lowest drawn point, in meters.
double floor_beneath(const scenario_grid &grid, const field_placement &placed);

// The height the level curve at `level` stands at, in meters.
double level_height(const map_placement &at, double level);

// Where x_k stands; nowhere for an x_k outside the ranges, nor on the field for one whose f was not
// answered finitely.
std::optional<Eigen::Vector3d> standing_at(const scenario_snapshot &shown, std::size_t k, const map_placement &at);

// The ends of the step from x_k to x_(k+1), where both stand, raised to the rod's axis
// `floor_step_lift` above a floor.
std::optional<std::array<Eigen::Vector3d, 2>> step_ends(const scenario_snapshot &shown, std::size_t k, const map_placement &at);

// grad f at x_k where x_k lies within the ranges and the gradient was answered finitely.
std::optional<Eigen::Vector2d> answered_gradient(const scenario_snapshot &shown, std::size_t k);

}

#endif
