#include "registered_values.h"

#include "praxis/optimization/registered.h"

#include <Eigen/Core>

#include <cmath>
#include <string>
#include <utility>
#include <optional>
#include <functional>

namespace praxis::optimization {

namespace {

bool inside(const Eigen::VectorXd &e, double tolerance)
{
    return e.size() == 1 && std::abs(e[0]) < tolerance;
}

}

registered_curve::registered_curve(std::string called, std::function<double(double)> g, std::function<double(double)> slope)
        : registered_curve(std::move(called), std::move(g), std::move(slope), std::function<double(double)>())
{
}

registered_curve::registered_curve(std::string called, std::function<double(double)> g, std::function<double(double)> slope, box region)
        : registered_curve(std::move(called), std::move(g), std::move(slope), std::function<double(double)>(), std::move(region))
{
}

registered_curve::registered_curve(std::string called, std::function<double(double)> g, std::function<double(double)> slope, std::function<double(double)> second)
        : name(std::move(called))
        , bounds(std::nullopt)
        , value(std::move(g))
        , derivative(std::move(slope))
        , second_derivative(std::move(second))
{
}

registered_curve::registered_curve(std::string called, std::function<double(double)> g, std::function<double(double)> slope, std::function<double(double)> second, box region)
        : name(std::move(called))
        , bounds(std::move(region))
        , value(std::move(g))
        , derivative(std::move(slope))
        , second_derivative(std::move(second))
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
        : registered_surface(std::move(called), std::move(f), std::move(slope), std::function<Eigen::Matrix2d(double, double)>())
{
}

registered_surface::registered_surface(std::string called, std::function<double(double, double)> f, std::function<Eigen::Vector2d(double, double)> slope, box region)
        : registered_surface(std::move(called), std::move(f), std::move(slope), std::function<Eigen::Matrix2d(double, double)>(), std::move(region))
{
}

registered_surface::registered_surface(std::string called, std::function<double(double, double)> f, std::function<Eigen::Vector2d(double, double)> slope,
                                       std::function<Eigen::Matrix2d(double, double)> curvature)
        : name(std::move(called))
        , bounds(std::nullopt)
        , value(std::move(f))
        , gradient(std::move(slope))
        , hessian(std::move(curvature))
{
}

registered_surface::registered_surface(std::string called, std::function<double(double, double)> f, std::function<Eigen::Vector2d(double, double)> slope,
                                       std::function<Eigen::Matrix2d(double, double)> curvature, box region)
        : name(std::move(called))
        , bounds(std::move(region))
        , value(std::move(f))
        , gradient(std::move(slope))
        , hessian(std::move(curvature))
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
