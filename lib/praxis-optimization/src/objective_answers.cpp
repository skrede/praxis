#include "objective_answers.h"

#include <optional>

namespace praxis::optimization {

namespace {

std::optional<refusal> judged(const problem &objective, const Eigen::VectorXd &answer)
{
    const expected<Eigen::VectorXd, refusal> g = objective.gradient(answer);
    if(g && objective.stationary(*g))
        return std::nullopt;

    return refusal::no_solution;
}

void record(const problem &objective, const problem &residual, scenario_run &ran)
{
    for(const Eigen::VectorXd &x : ran.iterates)
    {
        ran.errors.push_back(residual.error(x));
        ran.jacobians.push_back(residual.jacobian(x));
        ran.objectives.push_back(objective.objective(x));
        ran.gradients.push_back(objective.gradient(x));
        if(objective.hessian)
            ran.hessians.push_back(objective.hessian(x));
    }
}

}

void minimized(const problem &objective, const problem &residual, const expected<Eigen::VectorXd, refusal> &answered, scenario_run &ran)
{
    if(answered)
        ran.answer = *answered;
    ran.refused = answered ? judged(objective, *answered) : std::optional<refusal>(answered.error());
    if(ran.iterates.empty())
        ran.iterates.push_back(ran.seed);

    record(objective, residual, ran);
}

}
