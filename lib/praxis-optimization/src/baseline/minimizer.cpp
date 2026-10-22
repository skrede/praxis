#include "praxis/optimization/baseline/minimizer.h"

#include <Eigen/QR>

#include <cmath>

namespace praxis::optimization {

namespace {

// Steps from x0 by next(x, grad f(x)) until stationary holds of the gradient or max_iterations steps
// were taken.
template<typename Step>
expected<Eigen::VectorXd, refusal> descend(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates, Step next)
{
    Eigen::VectorXd x = x0;
    iterates.push_back(x);
    expected<Eigen::VectorXd, refusal> g = p.gradient(x);
    for(std::uint32_t k = 0; g && g->size() == x.size() && !p.stationary(*g) && k < max_iterations; ++k)
    {
        const expected<Eigen::VectorXd, refusal> stepped = next(x, *g);
        if(!stepped)
            return praxis::unexpected(stepped.error());

        x = *stepped;
        iterates.push_back(x);
        g = p.gradient(x);
    }
    if(!g)
        return praxis::unexpected(g.error());
    if(g->size() != x.size())
        return praxis::unexpected(refusal::degenerate);
    if(!p.stationary(*g))
        return praxis::unexpected(refusal::no_solution);

    return x;
}

}

// Nocedal & Wright, Numerical Optimization, 1st ed., sec. 2.2, eq. (2.14): the Newton direction
// p = -(grad^2 f)^-1 grad f. Lynch & Park, Modern Robotics, sec. 6.2.1 and App. D: the Newton-Raphson
// step on grad f = 0 with J = H, applied by complete orthogonal decomposition.
expected<Eigen::VectorXd, refusal> newton(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    if(!p.gradient || !p.hessian || !p.stationary)
        return praxis::unexpected(refusal::unsupported_input);

    return descend(p, x0, max_iterations, iterates,
                   [&p](const Eigen::VectorXd &x, const Eigen::VectorXd &g) -> expected<Eigen::VectorXd, refusal>
                   {
                       const expected<Eigen::MatrixXd, refusal> h = p.hessian(x);
                       if(!h || h->rows() != x.size() || h->cols() != x.size())
                           return praxis::unexpected(h ? refusal::degenerate : h.error());

                       return Eigen::VectorXd(x - Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>(*h).solve(g));
                   });
}

// Nocedal & Wright, Numerical Optimization, 1st ed., sec. 2.2: the steepest-descent direction
// -grad f, taken at a fixed step length.
expected<Eigen::VectorXd, refusal> gradient_descent(const problem &p, const Eigen::VectorXd &x0, double step_length, std::uint32_t max_iterations,
                                                    std::vector<Eigen::VectorXd> &iterates)
{
    if(!p.gradient || !p.stationary || !(std::isfinite(step_length) && step_length > 0.0))
        return praxis::unexpected(refusal::unsupported_input);

    return descend(p, x0, max_iterations, iterates,
                   [step_length](const Eigen::VectorXd &x, const Eigen::VectorXd &g) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd(x - step_length * g); });
}

}
