#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SOLVER_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SOLVER_H

#include "praxis/optimization/problem.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>

#include <vector>
#include <cstdint>

namespace praxis::optimization::inert {

expected<Eigen::VectorXd, refusal> solve(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates);

}

namespace praxis::optimization {

// A solve appends x0 first and every iterate after it to the list it is handed, which the caller hands
// empty. It answers the iterate at which converged holds of its error, and after max_iterations steps
// without that it refuses no_solution. A refusal from error or jacobian is answered as it came, and a
// Jacobian that is not e's length by x's is refused degenerate. The iterates reached are kept in every
// case. A member the solve reads that the problem leaves empty is refused as unsupported_input before
// anything is appended.
struct solver_ops
{
    expected<Eigen::VectorXd, refusal> (*solve)(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates) = &inert::solve;
};

}

#endif
