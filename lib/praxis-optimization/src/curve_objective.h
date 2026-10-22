#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_CURVE_OBJECTIVE_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_CURVE_OBJECTIVE_H

#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/plot_window.h"

#include <vector>

namespace praxis::optimization {

// The vertex of the quadratic model Newton's method minimized at the selected step. It is drawn before
// the span is taken, so the model is held to an ordinate range holding it.
void add_vertex(const scenario_snapshot &shown, std::vector<scene::plot_series> &drawn);

// The quadratic model at the selected step, sampled over the range and broken where it leaves the span,
// and the recorded x_(k+1) as a segment from the model's value there to f.
void add_newton_model(const scenario_snapshot &shown, const interval &span, std::vector<scene::plot_series> &drawn);

// The tangent of f at the selected step, and the recorded x_(k+1) as a segment from the tangent's value
// there to f.
void add_descent_tangent(const scenario_snapshot &shown, const interval &span, std::vector<scene::plot_series> &drawn);

}

#endif
