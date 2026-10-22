#include "registered_list.h"
#include "scenario_answers.h"
#include "scenario_defaults.h"
#include "scenario_admission.h"

#include "praxis/optimization/scenario_state.h"

#include <spdlog/spdlog.h>

#include <span>
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

std::uint32_t held_to(std::uint32_t step, const scenario_run &ran)
{
    const std::uint32_t taken = steps_taken(ran);

    return taken == 0u ? 0u : std::min(step, taken - 1u);
}

}

scenario_state::scenario_state(std::vector<registered_curve> curves, capabilities solving, scenario_settings opened, std::shared_ptr<scenario_publisher> published,
                               bool checks_derivative)
        : scenario_state(1u, names_of(curves), posed_all(curves, &curve_problem, curve_tolerance), solving, std::move(opened), std::move(published), checks_derivative)
{
}

scenario_state::scenario_state(std::vector<registered_surface> surfaces, capabilities solving, scenario_settings opened, std::shared_ptr<scenario_publisher> published,
                               bool checks_derivative)
        : scenario_state(2u, names_of(surfaces), posed_all(surfaces, &surface_problem, surface_tolerance), solving, std::move(opened), std::move(published), checks_derivative)
{
}

scenario_state::scenario_state(std::size_t dimension, std::vector<std::string> names, std::vector<problem> problems, capabilities solving, scenario_settings opened,
                               std::shared_ptr<scenario_publisher> published, bool checks_derivative)
        : m_checks(checks_derivative)
        , m_dimension(dimension)
        , m_solving(solving)
        , m_settings(std::move(opened))
        , m_names(std::move(names))
        , m_problems(std::move(problems))
        , m_point()
        , m_published(std::move(published))
{
}

void scenario_state::run()
{
    solve(m_settings.limit, m_settings.step);
}

void scenario_state::run(const Eigen::VectorXd &seed, std::uint32_t limit)
{
    if(!taken_at(seed, m_dimension, "solved from one finite seed"))
        return;

    m_settings.seed  = seed;
    m_settings.limit = limit;
    run();
}

void scenario_state::step_once(const Eigen::VectorXd &seed)
{
    if(!taken_at(seed, m_dimension, "solved from one finite seed"))
        return;

    const bool continuing     = m_shown != nullptr && m_shown->run.seed == seed;
    const std::uint32_t steps = continuing ? steps_taken(m_shown->run) + 1u : 1u;

    m_settings.seed = seed;
    solve(std::min(steps, m_settings.limit), steps - 1u);
}

void scenario_state::reset(const Eigen::VectorXd &seed)
{
    if(!taken_at(seed, m_dimension, "solved from one finite seed"))
        return;

    m_settings.seed = seed;
    solve(0u, 0u);
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
    if(!taken_at(point, m_dimension, "linearized at one finite point"))
        return;

    m_point = point;
    if(m_shown == nullptr)
        return;

    auto moved           = std::make_shared<scenario_snapshot>(*m_shown);
    moved->linearization = linearized_at(m_problems[moved->selected], point, m_checks);
    republish(std::move(moved));
}

void scenario_state::select(std::size_t index)
{
    if(index >= m_names.size())
    {
        spdlog::error("praxis: function {} was selected of the {} registered with this scenario; the last run stays shown", index + 1u, m_names.size());

        return;
    }

    m_settings.selected = m_names[index];
    run();
}

void scenario_state::set_range(std::span<const interval> ranges)
{
    if(!drawn_over(ranges, m_dimension))
        return;

    m_settings.ranges.assign(ranges.begin(), ranges.end());
    run();
}

void scenario_state::solve(std::uint32_t steps, std::uint32_t selecting)
{
    const std::optional<std::size_t> selected = selected_among(m_names, m_settings.selected);
    if(!selected)
        return;

    const problem &posed = m_problems[*selected];
    scenario_run ran     = solved(posed, m_solving.solver, m_settings.seed, steps);
    m_settings.step      = held_to(selecting, ran);

    auto shown = std::make_shared<scenario_snapshot>(
            scenario_snapshot{steps < m_settings.limit, m_settings.step, *selected, m_settings.ranges, {}, {}, nullptr, std::move(ran), posed.bounds, std::nullopt});
    if(m_dimension == 1u)
        sample(posed, m_settings.ranges.front(), shown->abscissa, shown->ordinate);
    else
        shown->grid = sampled_grid(posed, m_settings.ranges);
    if(m_point)
        shown->linearization = linearized_at(posed, *m_point, m_checks);
    republish(std::move(shown));
}

void scenario_state::republish(std::shared_ptr<scenario_snapshot> moved)
{
    m_shown = std::move(moved);
    m_published->publish(m_shown);
}

bool scenario_state::admits(const scenario_settings &edited) const
{
    return admitted(edited, m_names, m_dimension);
}

}
