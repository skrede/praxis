#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_CURVE_OVERLAYS_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_CURVE_OVERLAYS_H

#include "praxis/optimization/problem.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/plot_window.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>

#include <limits>
#include <string>
#include <vector>
#include <cstdint>
#include <optional>

namespace praxis::optimization {

inline constexpr double unanswered = std::numeric_limits<double>::quiet_NaN();

struct linearized
{
    double x;
    double g;
    double slope;
};

// NaN for a point other than of one coordinate.
double coordinate(const Eigen::VectorXd &x);

// g = -e and g' = J; an answer the problem refused stands at NaN and is not drawn.
double value_of(const expected<Eigen::VectorXd, refusal> &e);
double slope_of(const expected<Eigen::MatrixXd, refusal> &jacobian);

// The ordinates the plot is fitted to: those of finite points lying within the held range.
std::optional<interval> finite_span(const std::vector<scene::plot_series> &drawn, const interval &range);

// As wide as the part of the line within the range and the ordinates the plot is fitted to, centered on its point, and left out of the fit.
std::optional<scene::plot_series> tangent(std::string name, std::uint32_t tone, const linearized &at, const interval &range, const interval &span);

void add_step(const scenario_snapshot &shown, const interval &span, std::vector<scene::plot_series> &drawn);

// The point is drawn before the span is taken, so the span holds it.
std::optional<linearized> linearized_point(const std::optional<scenario_linearization> &at, std::vector<scene::plot_series> &drawn);

void add_linearization(const std::optional<scenario_linearization> &at, const linearized &here, const interval &range, const interval &span, std::vector<scene::plot_series> &drawn);

void add_bounds(const std::optional<box> &bounds, const interval &span, std::vector<scene::plot_series> &drawn);

}

#endif
