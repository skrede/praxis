#include "praxis/optimization/registered.h"

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
            .converged = [tolerance](const Eigen::VectorXd &e) -> bool { return e.size() == 1 && std::abs(e[0]) < tolerance; },
            .bounds    = curve.bounds,
    };
}

}
