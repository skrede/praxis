#include "scenario_answers.h"
#include "scenario_defaults.h"

#include <cmath>
#include <limits>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <algorithm>

namespace praxis::optimization {

namespace {

// Numerical Recipes, 3rd ed., sec. 5.7: h = cbrt(eps) max(1, |x_i|), divided by the difference of
// the two abscissae actually represented rather than by 2h.
std::optional<Eigen::MatrixXd> central_difference(const problem &posed, const Eigen::VectorXd &x)
{
    Eigen::MatrixXd estimated;
    for(Eigen::Index i = 0; i < x.size(); ++i)
    {
        const double h       = std::cbrt(std::numeric_limits<double>::epsilon()) * std::max(1.0, std::abs(x[i]));
        Eigen::VectorXd up   = x;
        Eigen::VectorXd down = x;
        up[i] += h;
        down[i] -= h;

        const expected<Eigen::VectorXd, refusal> above = posed.error(up);
        const expected<Eigen::VectorXd, refusal> below = posed.error(down);
        if(!above || !below || above->size() != below->size() || (i > 0 && above->size() != estimated.rows()))
            return std::nullopt;

        estimated.conservativeResize(above->size(), x.size());
        estimated.col(i) = (*below - *above) / (up[i] - down[i]);
    }

    return estimated;
}

double relative_difference(const Eigen::MatrixXd &given, const Eigen::MatrixXd &estimated)
{
    const Eigen::ArrayXXd scale = given.array().abs().max(estimated.array().abs()).max(1.0);

    return ((given - estimated).array().abs() / scale).maxCoeff();
}

}

scenario_run solved(const problem &posed, const solver_ops &solver, double seed, std::uint32_t steps)
{
    scenario_run ran{Eigen::VectorXd::Constant(1, seed), steps, {}, {}, {}, std::nullopt};

    const expected<Eigen::VectorXd, refusal> answered = solver.solve(posed, ran.seed, ran.limit, ran.iterates);
    if(!answered)
        ran.refused = answered.error();
    if(ran.iterates.empty())
        ran.iterates.push_back(ran.seed);

    for(const Eigen::VectorXd &x : ran.iterates)
    {
        ran.errors.push_back(posed.error(x));
        ran.jacobians.push_back(posed.jacobian(x));
    }

    return ran;
}

void sample(const problem &posed, const interval &range, std::vector<double> &abscissa, std::vector<double> &ordinate)
{
    const double step = (range.upper - range.lower) / static_cast<double>(curve_samples - 1);
    for(std::size_t at = 0; at < curve_samples; ++at)
    {
        const double x                             = at + 1 == curve_samples ? range.upper : range.lower + step * static_cast<double>(at);
        const expected<Eigen::VectorXd, refusal> e = posed.error(Eigen::VectorXd::Constant(1, x));

        abscissa.push_back(x);
        ordinate.push_back(e ? -(*e)[0] : std::numeric_limits<double>::quiet_NaN());
    }
}

scenario_linearization linearized_at(const problem &posed, const Eigen::VectorXd &point, bool checked)
{
    scenario_linearization at{point, posed.error(point), posed.jacobian(point), std::nullopt};
    if(!checked || !at.jacobian || at.jacobian->size() == 0)
        return at;

    const std::optional<Eigen::MatrixXd> estimated = central_difference(posed, point);
    if(!estimated || estimated->rows() != at.jacobian->rows() || estimated->cols() != at.jacobian->cols())
        return at;

    const double difference = relative_difference(*at.jacobian, *estimated);
    at.check                = derivative_check{*estimated, difference, difference > derivative_check_tolerance};

    return at;
}

}
