#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_STEP_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_STEP_H

#include "praxis/optimization/scenario_snapshot.h"

#include <Eigen/Core>

#include <span>
#include <array>
#include <vector>
#include <optional>

namespace praxis::optimization {

// x has one coordinate per range and lies within each.
bool within_ranges(std::span<const interval> ranges, const Eigen::VectorXd &x);

// The tangent plane z = f + g . (q - x) of a step from x, drawn as a square about x with sides along g
// and across it, of half-side `tangent_patch` times the distance from x to the plane's zero line and
// held to the ranges; the part of that zero line within the square, held to the ranges; and the point
// of the line nearest x, x - g f / |g|^2. Lynch & Park, Modern Robotics, sec. 6.2: the step J^+ e of a
// one-row J is the least-norm one, so that point is where the book's step lands. The patch is a convex
// polygon in order, each corner (x, y, z) in the function's coordinates.
struct step_construction
{
    std::vector<Eigen::Vector3d> patch;
    std::optional<std::array<Eigen::Vector2d, 2>> zero_line;
    Eigen::Vector2d nearest;
};

// Absent where the gradient is zero or anything handed in is not finite, and where x already stands
// on the zero line.
std::optional<step_construction> constructed_step(const Eigen::Vector2d &x, double f, const Eigen::Vector2d &gradient, std::span<const interval> ranges);

// The tangent plane at p over a rectangle with the ranges' widths, centered on p and held to the
// ranges: four corners in order, empty where nothing of it lies within them.
std::vector<Eigen::Vector3d> tangent_plane_over(const Eigen::Vector2d &p, double f, const Eigen::Vector2d &gradient, std::span<const interval> ranges);

}

#endif
