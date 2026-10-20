#include "praxis/optimization/scenario_state.h"

#include <spdlog/spdlog.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>

namespace praxis::optimization {

namespace {

constexpr std::size_t curve_samples = 1601;

std::optional<std::size_t> index_of(const std::vector<registered_curve> &curves, const std::string &name)
{
    const auto found = std::find_if(curves.begin(), curves.end(), [&name](const registered_curve &curve) { return curve.name == name; });
    if(found == curves.end())
        return std::nullopt;

    return static_cast<std::size_t>(found - curves.begin());
}

// A seed the solve did not record is still where the run started, so it is kept as the one iterate.
scenario_run solved(const problem &posed, const solver_ops &solver, const scenario_settings &settings)
{
    scenario_run ran{Eigen::VectorXd::Constant(1, settings.seed), settings.limit, {}, {}, {}, std::nullopt};

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

// g = -e, read back through the problem so a sample it refuses breaks the line rather than vanishing.
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

}

scenario_state::scenario_state(std::vector<registered_curve> curves, capabilities solving, scenario_settings opened, std::shared_ptr<scenario_publisher> published)
        : m_solving(solving)
        , m_settings(std::move(opened))
        , m_curves(std::move(curves))
        , m_published(std::move(published))
{
}

void scenario_state::run()
{
    const std::optional<std::size_t> selected = index_of(m_curves, m_settings.selected);
    if(!selected)
    {
        spdlog::error("praxis: no function named '{}' is registered with this scenario; the last run stays shown", m_settings.selected);
        return;
    }

    const problem posed = curve_problem(m_curves[*selected], m_settings.tolerance);

    auto ran = std::make_shared<scenario_snapshot>(scenario_snapshot{*selected, m_settings.range, {}, {}, solved(posed, m_solving.solver, m_settings)});
    sample(posed, m_settings.range, ran->abscissa, ran->ordinate);

    m_published->publish(std::move(ran));
}

}
