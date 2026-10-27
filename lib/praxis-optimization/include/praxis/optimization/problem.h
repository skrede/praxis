#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_PROBLEM_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_PROBLEM_H

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <functional>

namespace praxis::optimization {

// A coordinate whose bound is infinite is free.
struct box
{
    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
    // Values a whole period apart name the same point; empty, or an entry of zero, does not wrap.
    Eigen::VectorXd period = {};
};

// Lynch & Park, Modern Robotics, sec. 6.2: e(x) = x_d - f(x), J = df/dx, a step x += J^+ e.
// The objective form minimizes f: R^n -> R; stationary is tested over grad f.
struct problem
{
    std::function<expected<Eigen::VectorXd, refusal>(const Eigen::VectorXd &x)> error    = {};
    std::function<expected<Eigen::MatrixXd, refusal>(const Eigen::VectorXd &x)> jacobian = {};
    std::function<bool(const Eigen::VectorXd &e)> converged                              = {};
    std::optional<box> bounds                                                            = std::nullopt;
    std::function<expected<double, refusal>(const Eigen::VectorXd &x)> objective         = {};
    std::function<expected<Eigen::VectorXd, refusal>(const Eigen::VectorXd &x)> gradient = {};
    std::function<expected<Eigen::MatrixXd, refusal>(const Eigen::VectorXd &x)> hessian  = {};
    std::function<bool(const Eigen::VectorXd &gradient)> stationary                      = {};
};

// Whether a scenario's function is solved for a root or minimized. The order is the document's
// spellings', which is what reading a spelling back as an index and casting relies on.
enum class problem_form : std::uint8_t
{
    root,
    minimum
};

}

#endif
