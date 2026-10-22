#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_MINIMIZER_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_MINIMIZER_H

#include "praxis/optimization/problem.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>

#include <vector>
#include <cstdint>

namespace praxis::optimization::inert {

expected<Eigen::VectorXd, refusal> newton(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates);
expected<Eigen::VectorXd, refusal> gradient_descent(const problem &p, const Eigen::VectorXd &x0, double step_length, std::uint32_t max_iterations,
                                                    std::vector<Eigen::VectorXd> &iterates);

}

namespace praxis::optimization {

// A minimization appends x0 first and every iterate after it to the list it is handed, which the
// caller hands empty. It answers the iterate at which stationary holds of the gradient, and after
// max_iterations steps without that it refuses no_solution. A refusal from gradient or hessian is
// answered as it came, and a gradient that is not x's length or a Hessian that is not square in x's
// length is refused degenerate. The iterates reached are kept in every case. A member the
// minimization reads that the problem leaves empty, or a step length that is not finite and positive,
// is refused as unsupported_input before anything is appended. Gradient descent never reads hessian.
struct minimizer_ops
{
    expected<Eigen::VectorXd, refusal> (*newton)(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates) = &inert::newton;
    expected<Eigen::VectorXd, refusal> (*gradient_descent)(const problem &p, const Eigen::VectorXd &x0, double step_length, std::uint32_t max_iterations,
                                                           std::vector<Eigen::VectorXd> &iterates) = &inert::gradient_descent;
};

}

#endif
