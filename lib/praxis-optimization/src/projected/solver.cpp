#include "step_constants.h"

#include "praxis/optimization/projected/solver.h"

#include <Eigen/Cholesky>

#include <utility>
#include <optional>
#include <algorithm>

namespace praxis::optimization::projected {

namespace {

bool well_posed(const problem &p, const Eigen::VectorXd &x0)
{
    if(!p.error || !p.jacobian || !p.converged)
        return false;
    if(!p.bounds)
        return true;

    const box &bounds = *p.bounds;

    return bounds.lower.size() == x0.size() && bounds.upper.size() == x0.size() && (bounds.lower.array() <= bounds.upper.array()).all();
}

// Sugihara's damping at W_E = I: d = (J^T J + (E + bias) I)^-1 J^T e, E = 1/2 e^T e.
Eigen::VectorXd damped_step(const Eigen::MatrixXd &j, const Eigen::VectorXd &e)
{
    const Eigen::Index n               = j.cols();
    const Eigen::MatrixXd coefficients = j.transpose() * j + (0.5 * e.dot(e) + bias) * Eigen::MatrixXd::Identity(n, n);

    return coefficients.llt().solve(j.transpose() * e);
}

// The Euclidean projection onto a box is the componentwise clamp.
Eigen::VectorXd projected(const std::optional<box> &bounds, Eigen::VectorXd x)
{
    if(bounds)
        for(Eigen::Index i = 0; i < x.size(); ++i)
            x[i] = std::clamp(x[i], bounds->lower[i], bounds->upper[i]);

    return x;
}

// A projected point equal to the iterate it came from would repeat to the limit, so it refuses.
expected<Eigen::VectorXd, refusal> stepped(const problem &p, const Eigen::VectorXd &x, const Eigen::VectorXd &e)
{
    const expected<Eigen::MatrixXd, refusal> j = p.jacobian(x);
    if(!j)
        return praxis::unexpected(j.error());

    Eigen::VectorXd next = projected(p.bounds, x + damped_step(*j, e));
    if(next == x)
        return praxis::unexpected(refusal::no_solution);

    return next;
}

}

// x_{k+1} = P(x_k + d_k), the local version of Kanzow, Yamashita & Fukushima.
expected<Eigen::VectorXd, refusal> solve(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    if(!well_posed(p, x0))
        return praxis::unexpected(refusal::unsupported_input);

    Eigen::VectorXd x = x0;
    iterates.push_back(x);
    expected<Eigen::VectorXd, refusal> e = p.error(x);
    for(std::uint32_t k = 0; e && !p.converged(*e) && k < max_iterations; ++k)
    {
        expected<Eigen::VectorXd, refusal> next = stepped(p, x, *e);
        if(!next)
            return praxis::unexpected(next.error());

        x = std::move(*next);
        iterates.push_back(x);
        e = p.error(x);
    }
    if(!e)
        return praxis::unexpected(e.error());
    if(!p.converged(*e))
        return praxis::unexpected(refusal::no_solution);

    return x;
}

}
