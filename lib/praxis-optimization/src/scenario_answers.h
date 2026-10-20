#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_ANSWERS_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_ANSWERS_H

#include "praxis/optimization/problem.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/optimization/solver.h"

#include <Eigen/Core>

#include <vector>
#include <cstdint>

namespace praxis::optimization {

// A seed the solve did not record is still where the run started, so it is kept as the one iterate.
scenario_run solved(const problem &posed, const solver_ops &solver, double seed, std::uint32_t steps);

// g = -e at the curve's samples over the range, read back through the problem so a sample it refuses
// breaks the line rather than vanishing.
void sample(const problem &posed, const interval &range, std::vector<double> &abscissa, std::vector<double> &ordinate);

// The error and the Jacobian at `point`, and where `checked`, the Jacobian of f = -e estimated by
// central differences and compared with the one the problem answered.
scenario_linearization linearized_at(const problem &posed, const Eigen::VectorXd &point, bool checked);

}

#endif
