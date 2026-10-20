#include "praxis/optimization/baseline/solver.h"

#include <Eigen/QR>

namespace praxis::optimization {

// Newton-Raphson, Lynch & Park, Modern Robotics, sec. 6.2.1: x_{k+1} = x_k + J^+(x_k) e(x_k), the
// pseudoinverse applied by complete orthogonal decomposition.
expected<Eigen::VectorXd, refusal> solve(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    if(!p.error || !p.jacobian || !p.converged)
        return praxis::unexpected(refusal::unsupported_input);

    Eigen::VectorXd x = x0;
    iterates.push_back(x);
    expected<Eigen::VectorXd, refusal> e = p.error(x);
    for(std::uint32_t k = 0; e && !p.converged(*e) && k < max_iterations; ++k)
    {
        const expected<Eigen::MatrixXd, refusal> j = p.jacobian(x);
        if(!j || j->rows() != e->size() || j->cols() != x.size())
            return praxis::unexpected(j ? refusal::degenerate : j.error());

        x += Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>(*j).solve(*e);
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
