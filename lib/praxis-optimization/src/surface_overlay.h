#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_OVERLAY_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_OVERLAY_H

#include "surface_mesh.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/scenario_snapshot.h"

#include <threepp/objects/Mesh.hpp>

#include <threepp/core/Object3D.hpp>

#include <Eigen/Core>

#include <span>
#include <vector>
#include <optional>

namespace praxis::optimization {

// What the selected step draws: the patch of the tangent plane at x_k, its zero line and the point of
// that line nearest x_k on the zero plane, and the recorded x_(k+1) as a line from the zero plane up
// to f there. Nothing is drawn from an iterate outside the ranges or one whose f or gradient was not
// answered.
struct step_figures
{
    threepp::Mesh &patch;
    threepp::Object3D &zero_line;
    threepp::Object3D &nearest;
    threepp::Object3D &recorded;
};

void draw_step(const step_figures &drawn, const scenario_snapshot &shown, const field_placement &placed);

// The tangent plane at the Linearization point, and where the check disagrees, the plane of the
// central difference, each over the ranges and held to the field's heights and f(p).
struct point_figures
{
    threepp::Mesh &plane;
    threepp::Mesh &central;
};

void draw_planes(const point_figures &drawn, const scenario_snapshot &shown, const field_placement &placed, const interval &heights);

// The box's four sides on the zero plane as pairs of points, each bound held to its range, so an
// infinite one stands at the range's edge; nothing where the box leaves no area within the ranges.
std::vector<Eigen::Vector3d> box_outline(const std::optional<box> &bounds, std::span<const interval> ranges);

// The least and the greatest finite sample; both zero where none is finite.
interval height_span(const scenario_grid &grid);

}

#endif
