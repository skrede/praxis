#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_BASELINE_MINIMIZER_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_BASELINE_MINIMIZER_H

#include "praxis/optimization/minimizer.h"

#include <vector>
#include <cstdint>

// The declarations below match the slots of minimizer_ops by name and by signature, so composing the
// aggregate is a plain address-of.
namespace praxis::optimization {

expected<Eigen::VectorXd, refusal> newton(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates);
expected<Eigen::VectorXd, refusal> gradient_descent(const problem &p, const Eigen::VectorXd &x0, double step_length, std::uint32_t max_iterations,
                                                    std::vector<Eigen::VectorXd> &iterates);

}

#endif
