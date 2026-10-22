#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_OBJECTIVE_ANSWERS_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_OBJECTIVE_ANSWERS_H

#include "praxis/optimization/problem.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>

namespace praxis::optimization {

// Completes a run a minimizer has filled the iterates of and answered. A seed the minimization did
// not record is kept as the one iterate. The answer is judged by the objective's own stationary test
// rather than taken on the minimizer's word; the residual's error and Jacobian are kept beside f, its
// gradient and, where the objective carries one, its Hessian at every iterate.
void minimized(const problem &objective, const problem &residual, const expected<Eigen::VectorXd, refusal> &answered, scenario_run &ran);

}

#endif
