#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_ANSWERS_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_ANSWERS_H

#include "praxis/optimization/problem.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/optimization/solver.h"

#include <Eigen/Core>

#include <span>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>

namespace praxis::optimization {

// What a window shows before the first publication.
inline constexpr const char *nothing_published = "The function has not been evaluated yet.";

std::uint32_t steps_taken(const scenario_run &ran);

// The point `at` of `count` evenly spaced over the range, its ends exactly the range's.
double evenly(const interval &range, std::size_t at, std::size_t count);

// A seed the solve did not record is still where the run started, so it is kept as the one iterate.
// An answer is judged by the problem's own converged test rather than taken on the solver's word.
scenario_run solved(const problem &posed, const solver_ops &solver, const Eigen::VectorXd &seed, std::uint32_t steps);

// g = -e at the curve's samples over the range, read back through the problem so a sample it refuses
// breaks the line rather than vanishing.
void sample(const problem &posed, const interval &range, std::vector<double> &abscissa, std::vector<double> &ordinate);

// f = -e on the grid over the two ranges, read back through the problem as the curve's samples are.
std::shared_ptr<const scenario_grid> sampled_grid(const problem &posed, std::span<const interval> ranges);

// The error and the Jacobian at `point`, and where `checked`, the Jacobian of f = -e estimated by
// central differences and compared with the one the problem answered.
scenario_linearization linearized_at(const problem &posed, const Eigen::VectorXd &point, bool checked);

}

#endif
