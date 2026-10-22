#include "scenario_answers.h"
#include "objective_answers.h"
#include "scenario_admission.h"

#include "praxis/optimization/minimizer.h"
#include "praxis/optimization/scenario_state.h"

#include <cstddef>
#include <cstdint>
#include <optional>

namespace praxis::optimization {

namespace {

expected<Eigen::VectorXd, refusal> minimize(const minimizer_ops &minimizer, const problem &objective, const objective_settings &chosen, scenario_run &ran)
{
    switch(chosen.method)
    {
        case minimizer_slot::newton:
            return minimizer.newton(objective, ran.seed, ran.limit, ran.iterates);
        case minimizer_slot::gradient_descent:
            return minimizer.gradient_descent(objective, ran.seed, chosen.step_length, ran.limit, ran.iterates);
        default:
            return unexpected(refusal::unsupported_input);
    }
}

}

scenario_run scenario_state::posed_run(std::size_t selected, std::uint32_t steps) const
{
    if(m_settings.objective.form != problem_form::minimum)
        return solved(m_problems[selected], m_solving.solver, m_settings.seed, steps);

    scenario_run ran{m_settings.seed, steps, {}, {}, {}, std::nullopt, std::nullopt, {}, {}, {}, m_settings.objective.method};
    const expected<Eigen::VectorXd, refusal> answered = minimize(m_solving.minimizer, m_objectives[selected], m_settings.objective, ran);
    minimized(m_objectives[selected], m_problems[selected], answered, ran);

    return ran;
}

void scenario_state::select_method(minimizer_slot method)
{
    m_settings.objective.method = method;
    run();
}

void scenario_state::set_step_length(double step_length)
{
    if(!descends_by(step_length))
        return;

    m_settings.objective.step_length = step_length;
    run();
}

}
