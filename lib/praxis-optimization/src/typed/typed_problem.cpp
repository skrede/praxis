#include "typed_problem.h"

#include "jet.h"

#include <Eigen/Core>

#include <cmath>
#include <memory>

namespace praxis::optimization::typed {
namespace {

using tree_pointer = std::shared_ptr<const expression>;

bool fits(const tree_pointer &tree, const Eigen::VectorXd &x)
{
    return tree && x.size() == static_cast<Eigen::Index>(tree->dimension);
}

expected<double, refusal> value_at(const tree_pointer &tree, const parameter_values &values, const Eigen::VectorXd &x)
{
    if(!fits(tree, x))
        return unexpected(refusal::unsupported_input);

    const double f = evaluated<double>(*tree, x, values);
    if(!std::isfinite(f))
        return unexpected(refusal::degenerate);
    return f;
}

expected<Eigen::VectorXd, refusal> gradient_at(const tree_pointer &tree, const parameter_values &values, const Eigen::VectorXd &x)
{
    if(!fits(tree, x))
        return unexpected(refusal::unsupported_input);

    const Eigen::VectorXd g = evaluated<jet>(*tree, x, values).gradient.head(x.size());
    if(!g.allFinite())
        return unexpected(refusal::degenerate);
    return g;
}

expected<Eigen::MatrixXd, refusal> hessian_at(const tree_pointer &tree, const parameter_values &values, const Eigen::VectorXd &x)
{
    if(!fits(tree, x))
        return unexpected(refusal::unsupported_input);

    const Eigen::MatrixXd h = evaluated<jet>(*tree, x, values).hessian.topLeftCorner(x.size(), x.size());
    if(!h.allFinite())
        return unexpected(refusal::degenerate);
    return h;
}

}

problem typed_residual(std::shared_ptr<const expression> tree, parameter_values values, double tolerance)
{
    return problem{
            .error = [tree, values](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal>
            {
                const expected<double, refusal> f = value_at(tree, values, x);
                if(!f)
                    return unexpected(f.error());
                return Eigen::VectorXd::Constant(1, -*f);
            },
            .jacobian = [tree, values](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal>
            {
                const expected<Eigen::VectorXd, refusal> g = gradient_at(tree, values, x);
                if(!g)
                    return unexpected(g.error());
                return Eigen::MatrixXd(g->transpose());
            },
            .converged = [tolerance](const Eigen::VectorXd &e) -> bool { return e.size() == 1 && std::abs(e[0]) < tolerance; },
    };
}

problem typed_objective(std::shared_ptr<const expression> tree, parameter_values values, double tolerance)
{
    const Eigen::Index coordinates = tree ? static_cast<Eigen::Index>(tree->dimension) : 0;
    return problem{
            .objective  = [tree, values](const Eigen::VectorXd &x) { return value_at(tree, values, x); },
            .gradient   = [tree, values](const Eigen::VectorXd &x) { return gradient_at(tree, values, x); },
            .hessian    = [tree, values](const Eigen::VectorXd &x) { return hessian_at(tree, values, x); },
            .stationary = [coordinates, tolerance](const Eigen::VectorXd &g) -> bool { return g.size() == coordinates && (g.array().abs() < tolerance).all(); },
    };
}

}
