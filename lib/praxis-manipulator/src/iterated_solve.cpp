#include "joint_bounds.h"
#include "iterated_solve.h"

#include "baseline/chain_maps.h"

#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include <vector>
#include <cstdint>
#include <utility>
#include <optional>

namespace praxis::manipulator {

namespace {

bool within(const twist &error, const solver_parameters &parameters)
{
    return error.head<3>().norm() <= parameters.orientation_tol && error.tail<3>().norm() <= parameters.position_tol;
}

void record(ik_result &answer, const joint_vector &solution, const twist &error, const joint_vector &previous)
{
    answer.iterations.push_back(
            iteration_state{solution, error.head<3>().norm(), error.tail<3>().norm(), (solution - previous).norm(), static_cast<std::uint32_t>(answer.iterations.size())});
}

// Steps from the seed until both halves of V_b are within the handed tolerances, checked before each
// step and once after the last; each step is recorded at the configuration it reaches.
expected<joint_vector, refusal> stepped(const body_target &target, const joint_vector &j0, const solver_parameters &parameters, solve_step step, ik_result &answer)
{
    joint_vector theta             = j0;
    expected<twist, refusal> error = body_error(target, theta);
    for(std::uint32_t i = 0; error && !within(*error, parameters) && i < parameters.max_iterations_per_attempt; ++i)
    {
        const expected<jacobian, refusal> jb = body_columns(target.body, theta);
        if(!jb)
            return unexpected(refusal::no_solution);

        const expected<joint_vector, refusal> next = step(target, theta, *jb, *error);
        if(!next || next->size() != theta.size())
            return unexpected(refusal::no_solution);

        const joint_vector previous = std::exchange(theta, *next);
        error                       = body_error(target, theta);
        if(error)
            record(answer, theta, *error, previous);
    }
    if(!error || !within(*error, parameters) || !theta.allFinite())
        return unexpected(refusal::no_solution);

    return theta;
}

// A converged configuration is answered at its naming inside the chain's joint bounds.
expected<void, refusal> answered_inside_bounds(const screw_chain &chain, const expected<joint_vector, refusal> &converged, ik_result &answer)
{
    if(!converged)
        return unexpected(converged.error());

    const std::optional<joint_vector> named = named_inside_bounds(chain, *converged);
    if(!named.has_value())
        return unexpected(refusal::no_solution);

    answer.solutions.push_back(*named);

    return {};
}

expected<std::vector<screw_axis>, refusal> admitted_body(const screw_chain &chain, const transform &desired, const joint_vector &j0)
{
    if(j0.size() != static_cast<Eigen::Index>(chain.joint_count()))
        return unexpected(refusal::unsupported_input);
    if(!is_admitted(chain) || !is_a_rigid_motion(desired))
        return unexpected(refusal::degenerate);
    if(!j0.allFinite())
        return unexpected(refusal::no_solution);

    expected<std::vector<screw_axis>, refusal> body = to_body_screws(rigid_motion::baseline().screw, chain.home, chain.space_screws);
    if(!body)
        return unexpected(refusal::degenerate);

    return body;
}

}

expected<twist, refusal> body_error(const body_target &target, const joint_vector &theta)
{
    const transform reached                                          = target.chain.home * exponential_product(target.body, theta);
    const expected<std::pair<screw_axis, double>, refusal> logarithm = rigid_motion::matrix_logarithm_se3(transform(rigid_motion::inverse(reached) * target.desired));
    if(!logarithm)
        return unexpected(logarithm.error());

    return twist(logarithm->first * logarithm->second);
}

expected<void, refusal> iterated(const screw_chain &chain, const transform &desired, const joint_vector &j0, const solver_parameters &parameters, solve_step step, ik_result &answer)
{
    const expected<std::vector<screw_axis>, refusal> body = admitted_body(chain, desired, j0);
    if(!body)
        return unexpected(body.error());

    const body_target target{chain, desired, *body};

    return answered_inside_bounds(chain, stepped(target, j0, parameters, step, answer), answer);
}

}
