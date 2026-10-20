#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_BASELINE_SOLVER_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_BASELINE_SOLVER_H

#include "praxis/optimization/solver.h"

#include <vector>
#include <cstdint>

// The declaration below matches the slot of solver_ops by name and by signature, so composing the
// aggregate is a plain address-of.
namespace praxis::optimization {

expected<Eigen::VectorXd, refusal> solve(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates);

}

#endif
