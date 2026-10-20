#include "scenario_answers.h"
#include "scenario_defaults.h"

#include <span>
#include <cmath>
#include <limits>
#include <memory>
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

std::optional<refusal> judged(const problem &posed, const Eigen::VectorXd &answer)
{
    const expected<Eigen::VectorXd, refusal> e = posed.error(answer);
    if(e && posed.converged(*e))
        return std::nullopt;

    return refusal::no_solution;
}

double relative_difference(const Eigen::MatrixXd &given, const Eigen::MatrixXd &estimated)
{
    const Eigen::ArrayXXd scale = given.array().abs().max(estimated.array().abs()).max(1.0);

    return ((given - estimated).array().abs() / scale).maxCoeff();
}

}

std::uint32_t steps_taken(const scenario_run &ran)
{
    return static_cast<std::uint32_t>(ran.iterates.size() - 1u);
}

double evenly(const interval &range, std::size_t at, std::size_t count)
{
    if(at + 1 == count)
        return range.upper;

    return range.lower + (range.upper - range.lower) / static_cast<double>(count - 1) * static_cast<double>(at);
}

scenario_run solved(const problem &posed, const solver_ops &solver, const Eigen::VectorXd &seed, std::uint32_t steps)
{
    scenario_run ran{seed, steps, {}, {}, {}, std::nullopt, std::nullopt};

    const expected<Eigen::VectorXd, refusal> answered = solver.solve(posed, ran.seed, ran.limit, ran.iterates);
    if(answered)
        ran.answer = *answered;
    ran.refused = answered ? judged(posed, *answered) : std::optional<refusal>(answered.error());
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
    for(std::size_t at = 0; at < curve_samples; ++at)
    {
        const double x                             = evenly(range, at, curve_samples);
        const expected<Eigen::VectorXd, refusal> e = posed.error(Eigen::VectorXd::Constant(1, x));

        abscissa.push_back(x);
        ordinate.push_back(e ? -(*e)[0] : std::numeric_limits<double>::quiet_NaN());
    }
}

std::shared_ptr<const scenario_grid> sampled_grid(const problem &posed, std::span<const interval> ranges)
{
    auto grid = std::make_shared<scenario_grid>(scenario_grid{surface_samples, {}});
    grid->heights.reserve(surface_samples * surface_samples);
    for(std::size_t j = 0; j < surface_samples; ++j)
        for(std::size_t i = 0; i < surface_samples; ++i)
        {
            const expected<Eigen::VectorXd, refusal> e = posed.error(Eigen::Vector2d(evenly(ranges[0], i, surface_samples), evenly(ranges[1], j, surface_samples)));
            grid->heights.push_back(e ? -(*e)[0] : std::numeric_limits<double>::quiet_NaN());
        }

    return grid;
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
