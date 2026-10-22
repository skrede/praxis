#include "registered_values.h"

#include "praxis/optimization/registered.h"

#include <Eigen/Core>

#include <functional>

namespace praxis::optimization {

namespace {

using hessian_answer = std::function<expected<Eigen::MatrixXd, refusal>(const Eigen::VectorXd &x)>;

std::function<bool(const Eigen::VectorXd &)> stationary_below(Eigen::Index coordinates, double tolerance)
{
    return [coordinates, tolerance](const Eigen::VectorXd &g) -> bool { return g.size() == coordinates && (g.array().abs() < tolerance).all(); };
}

// Empty where nothing was registered, so the problem carries no Hessian.
hessian_answer hessian_of(const std::function<double(double)> &second)
{
    if(!second)
        return {};

    return [second](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal>
    {
        const expected<double, refusal> at = evaluated(second, x);
        if(!at)
            return unexpected(at.error());

        return Eigen::MatrixXd::Constant(1, 1, *at);
    };
}

hessian_answer hessian_of(const std::function<Eigen::Matrix2d(double, double)> &curvature)
{
    if(!curvature)
        return {};

    return [curvature](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal>
    {
        const expected<Eigen::Matrix2d, refusal> at = evaluated(curvature, x);
        if(!at)
            return unexpected(at.error());

        return Eigen::MatrixXd(*at);
    };
}

}

problem curve_objective(const registered_curve &curve, double tolerance)
{
    return problem{
            .bounds    = curve.bounds,
            .objective = [f = curve.value](const Eigen::VectorXd &x) -> expected<double, refusal> { return evaluated(f, x); },
            .gradient  = [slope = curve.derivative](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal>
            {
                const expected<double, refusal> at = evaluated(slope, x);
                if(!at)
                    return unexpected(at.error());

                return Eigen::VectorXd::Constant(1, *at);
            },
            .hessian    = hessian_of(curve.second_derivative),
            .stationary = stationary_below(1, tolerance),
    };
}

problem surface_objective(const registered_surface &surface, double tolerance)
{
    return problem{
            .bounds    = surface.bounds,
            .objective = [f = surface.value](const Eigen::VectorXd &x) -> expected<double, refusal> { return evaluated(f, x); },
            .gradient  = [slope = surface.gradient](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal>
            {
                const expected<Eigen::Vector2d, refusal> at = evaluated(slope, x);
                if(!at)
                    return unexpected(at.error());

                return Eigen::VectorXd(*at);
            },
            .hessian    = hessian_of(surface.hessian),
            .stationary = stationary_below(2, tolerance),
    };
}

}
