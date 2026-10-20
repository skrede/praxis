#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_PROJECTED_SOLVER_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_PROJECTED_SOLVER_H

#include "praxis/optimization/solver.h"

#include <vector>
#include <cstdint>

// The projected Levenberg-Marquardt method of Kanzow, Yamashita & Fukushima (J. Comput. Appl. Math. 172,
// 2004), local version, damped as Sugihara (IEEE Trans. Robotics 27(5), 2011). The declaration below
// matches the slot of solver_ops by signature. Every iterate after the handed seed lies inside the
// problem's box. A box of another width than the seed, or with a lower bound above its upper, is
// refused unsupported_input, and a step the box returns to the iterate it left is refused no_solution.
namespace praxis::optimization::projected {

expected<Eigen::VectorXd, refusal> solve(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates);

}

#endif
