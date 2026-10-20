#include "scenario_answers.h"
#include "scenario_defaults.h"

#include "praxis/optimization/scenario_state.h"

#include <spdlog/spdlog.h>

#include <span>
#include <cmath>
#include <format>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <optional>
#include <algorithm>

namespace praxis::optimization {

namespace {

std::optional<std::size_t> index_of(const std::vector<registered_curve> &curves, const std::string &name)
{
    const auto found = std::find_if(curves.begin(), curves.end(), [&name](const registered_curve &curve) { return curve.name == name; });
    if(found == curves.end())
        return std::nullopt;

    return static_cast<std::size_t>(found - curves.begin());
}

std::uint32_t steps_taken(const scenario_run &ran)
{
    return static_cast<std::uint32_t>(ran.iterates.size() - 1u);
}

std::uint32_t held_to(std::uint32_t step, const scenario_run &ran)
{
    const std::uint32_t taken = steps_taken(ran);

    return taken == 0u ? 0u : std::min(step, taken - 1u);
}

}

scenario_state::scenario_state(std::vector<registered_curve> curves, capabilities solving, scenario_settings opened, std::shared_ptr<scenario_publisher> published,
                               bool checks_derivative)
        : m_checks(checks_derivative)
        , m_solving(solving)
        , m_settings(std::move(opened))
        , m_point()
        , m_curves(std::move(curves))
        , m_published(std::move(published))
{
}

void scenario_state::run()
{
    solve(m_settings.limit, m_settings.step);
}

void scenario_state::run(const Eigen::VectorXd &seed, std::uint32_t limit)
{
    if(!takes(seed, "solved from one finite seed"))
        return;

    m_settings.seed  = seed[0];
    m_settings.limit = limit;
    run();
}

void scenario_state::step_once(const Eigen::VectorXd &seed)
{
    if(!takes(seed, "solved from one finite seed"))
        return;

    const bool continuing     = m_shown != nullptr && m_shown->run.seed == seed;
    const std::uint32_t steps = continuing ? steps_taken(m_shown->run) + 1u : 1u;

    m_settings.seed = seed[0];
    solve(std::min(steps, m_settings.limit), steps - 1u);
}

void scenario_state::select_step(std::uint32_t step)
{
    m_settings.step = step;
    if(m_shown == nullptr)
        return;

    auto moved      = std::make_shared<scenario_snapshot>(*m_shown);
    moved->step     = held_to(step, moved->run);
    m_settings.step = moved->step;
    republish(std::move(moved));
}

void scenario_state::set_limit(std::uint32_t limit)
{
    m_settings.limit = limit;
}

void scenario_state::linearize(const Eigen::VectorXd &point)
{
    if(!takes(point, "linearized at one finite point"))
        return;

    m_point = point;
    if(m_shown == nullptr)
        return;

    auto moved           = std::make_shared<scenario_snapshot>(*m_shown);
    moved->linearization = linearized_at(curve_problem(m_curves[moved->selected], curve_tolerance), point, m_checks);
    republish(std::move(moved));
}

void scenario_state::select(std::size_t index)
{
    if(index >= m_curves.size())
    {
        spdlog::error("praxis: function {} was selected of the {} registered with this scenario; the last run stays shown", index + 1u, m_curves.size());

        return;
    }

    m_settings.selected = m_curves[index].name;
    run();
}

void scenario_state::set_range(std::span<const interval> ranges)
{
    if(ranges.size() != 1u || !std::isfinite(ranges[0].lower) || !std::isfinite(ranges[0].upper) || !(ranges[0].lower < ranges[0].upper))
    {
        spdlog::error("praxis: a function of one variable is drawn over one finite range whose minimum lies below its maximum; the last run stays shown");

        return;
    }

    m_settings.range = ranges[0];
    run();
}

void scenario_state::solve(std::uint32_t steps, std::uint32_t selecting)
{
    const std::optional<std::size_t> selected = index_of(m_curves, m_settings.selected);
    if(!selected)
    {
        spdlog::error("praxis: no function named '{}' is registered with this scenario; the last run stays shown", m_settings.selected);
        return;
    }

    const problem posed = curve_problem(m_curves[*selected], curve_tolerance);
    scenario_run ran    = solved(posed, m_solving.solver, m_settings.seed, steps);
    m_settings.step     = held_to(selecting, ran);

    auto shown = std::make_shared<scenario_snapshot>(
            scenario_snapshot{steps < m_settings.limit, m_settings.step, *selected, m_settings.range, {}, {}, std::move(ran), posed.bounds, std::nullopt});
    sample(posed, m_settings.range, shown->abscissa, shown->ordinate);
    if(m_point)
        shown->linearization = linearized_at(posed, *m_point, m_checks);
    republish(std::move(shown));
}

void scenario_state::republish(std::shared_ptr<scenario_snapshot> moved)
{
    m_shown = std::move(moved);
    m_published->publish(m_shown);
}

bool scenario_state::takes(const Eigen::VectorXd &x, const char *as) const
{
    if(x.size() == 1 && std::isfinite(x[0]))
        return true;

    const std::string handed = x.size() == 1 ? std::string("a value that is not finite") : std::format("a vector of {} values", x.size());
    spdlog::error("praxis: a function of one variable is {}, which {} is not; the last run stays shown", as, handed);

    return false;
}

}
