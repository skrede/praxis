#include "praxis/optimization/registered.h"

#include <Eigen/Core>

#include <cmath>
#include <string>
#include <utility>
#include <optional>
#include <functional>

namespace praxis::optimization {

namespace {

expected<double, refusal> evaluated(const std::function<double(double)> &f, const Eigen::VectorXd &x)
{
    if(!f || x.size() != 1)
        return unexpected(refusal::unsupported_input);

    const double answered = f(x[0]);
    if(!std::isfinite(answered))
        return unexpected(refusal::degenerate);

    return answered;
}

expected<Eigen::Vector2d, refusal> evaluated(const std::function<Eigen::Vector2d(double, double)> &f, const Eigen::VectorXd &x)
{
    if(!f || x.size() != 2)
        return unexpected(refusal::unsupported_input);

    const Eigen::Vector2d answered = f(x[0], x[1]);
    if(!answered.allFinite())
        return unexpected(refusal::degenerate);

    return answered;
}

expected<double, refusal> evaluated(const std::function<double(double, double)> &f, const Eigen::VectorXd &x)
{
    if(!f || x.size() != 2)
        return unexpected(refusal::unsupported_input);

    const double answered = f(x[0], x[1]);
    if(!std::isfinite(answered))
        return unexpected(refusal::degenerate);

    return answered;
}

bool inside(const Eigen::VectorXd &e, double tolerance)
{
    return e.size() == 1 && std::abs(e[0]) < tolerance;
}

}

registered_curve::registered_curve(std::string called, std::function<double(double)> g, std::function<double(double)> slope)
        : name(std::move(called))
        , bounds(std::nullopt)
        , value(std::move(g))
        , derivative(std::move(slope))
{
}

registered_curve::registered_curve(std::string called, std::function<double(double)> g, std::function<double(double)> slope, box region)
        : name(std::move(called))
        , bounds(std::move(region))
        , value(std::move(g))
        , derivative(std::move(slope))
{
}

problem curve_problem(const registered_curve &curve, double tolerance)
{
    return problem{
            .error = [g = curve.value](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal>
            {
                const expected<double, refusal> at = evaluated(g, x);
                if(!at)
                    return unexpected(at.error());

                return Eigen::VectorXd::Constant(1, -*at);
            },
            .jacobian = [slope = curve.derivative](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal>
            {
                const expected<double, refusal> at = evaluated(slope, x);
                if(!at)
                    return unexpected(at.error());

                return Eigen::MatrixXd::Constant(1, 1, *at);
            },
            .converged = [tolerance](const Eigen::VectorXd &e) -> bool { return inside(e, tolerance); },
            .bounds    = curve.bounds,
    };
}

registered_surface::registered_surface(std::string called, std::function<double(double, double)> f, std::function<Eigen::Vector2d(double, double)> slope)
        : name(std::move(called))
        , bounds(std::nullopt)
        , value(std::move(f))
        , gradient(std::move(slope))
{
}

registered_surface::registered_surface(std::string called, std::function<double(double, double)> f, std::function<Eigen::Vector2d(double, double)> slope, box region)
        : name(std::move(called))
        , bounds(std::move(region))
        , value(std::move(f))
        , gradient(std::move(slope))
{
}

problem surface_problem(const registered_surface &surface, double tolerance)
{
    return problem{
            .error = [f = surface.value](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal>
            {
                const expected<double, refusal> at = evaluated(f, x);
                if(!at)
                    return unexpected(at.error());

                return Eigen::VectorXd::Constant(1, -*at);
            },
            .jacobian = [slope = surface.gradient](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal>
            {
                const expected<Eigen::Vector2d, refusal> at = evaluated(slope, x);
                if(!at)
                    return unexpected(at.error());

                return Eigen::MatrixXd(at->transpose());
            },
            .converged = [tolerance](const Eigen::VectorXd &e) -> bool { return inside(e, tolerance); },
            .bounds    = surface.bounds,
    };
}

}
