#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_LEVEL_CURVE_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_LEVEL_CURVE_H

#include "praxis/optimization/scenario_snapshot.h"

#include <Eigen/Core>

#include <span>
#include <array>
#include <vector>

namespace praxis::optimization {

using level_segment = std::array<Eigen::Vector2d, 2>;

// Where the sampled function crosses `level`, as segments in the coordinates of the ranges: marching
// squares (Lorensen & Cline, 1987, the two-dimensional case) with linear interpolation along each
// cell's edges, a saddle cell split by the mean of its four corners. A cell with a corner that is not
// finite contributes nothing, and a cell and its neighbor place their shared crossing at the same point.
std::vector<level_segment> level_curve(const scenario_grid &grid, std::span<const interval> ranges, double level);

}

#endif
