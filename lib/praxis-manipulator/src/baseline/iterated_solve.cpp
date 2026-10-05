#include "chain_maps.h"
#include "iterated_solve.h"

#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/evaluation/tolerance.h"

#include <span>
#include <cmath>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <utility>
#include <optional>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// V_b = log(T_sb(theta)^-1 T_sd), T_sb = M e^[B_1]theta_1 ... e^[B_n]theta_n: Lynch & Park, Modern
// Robotics, sec. 6.2.2 and eq. (4.16). The angular half leads.
expected<twist, refusal> body_error(const transform &home, std::span<const screw_axis> body, const transform &desired, const joint_vector &theta)
{
    const transform reached                                          = home * exponential_product(body, theta);
    const expected<std::pair<screw_axis, double>, refusal> logarithm = rigid_motion::matrix_logarithm_se3(transform(rigid_motion::inverse(reached) * desired));
    if(!logarithm)
        return unexpected(logarithm.error());

    return twist(logarithm->first * logarithm->second);
}

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
expected<joint_vector, refusal> iterated(const transform &home, std::span<const screw_axis> body, const transform &desired, const joint_vector &j0, const solver_parameters &parameters,
                                         solve_step step, ik_result &answer)
{
    joint_vector theta             = j0;
    expected<twist, refusal> error = body_error(home, body, desired, theta);
    for(std::uint32_t i = 0; error && !within(*error, parameters) && i < parameters.max_iterations_per_attempt; ++i)
    {
        const expected<jacobian, refusal> jb = body_columns(body, theta);
        if(!jb)
            return unexpected(refusal::no_solution);

        const joint_vector previous = theta;
        theta += step(*jb, *error);
        error = body_error(home, body, desired, theta);
        if(error)
            record(answer, theta, *error, previous);
    }
    if(!error || !within(*error, parameters) || !theta.allFinite())
        return unexpected(refusal::no_solution);

    return theta;
}

// Lynch & Park, Modern Robotics, Def. 3.24: a unit screw of pitch w . v = 0 is a pure rotation, so a
// whole turn about it is no displacement.
bool turns_whole(const screw_axis &s)
{
    return is_approx_equal(s.head<3>().norm(), 1.0) && is_approx_equal(s.head<3>().dot(s.tail<3>()), 0.0);
}

// value + k turn inside [lower, upper], k nearest zero; a turn of zero admits the value alone.
std::optional<double> named_between(double value, double lower, double upper, double turn)
{
    if(turn == 0.0)
        return lower <= value && value <= upper ? std::optional<double>(value) : std::nullopt;

    const double fewest = std::ceil((lower - value) / turn);
    const double most   = std::floor((upper - value) / turn);
    if(fewest > most)
        return std::nullopt;

    return value + turn * std::clamp(0.0, fewest, most);
}

// A joint that turns whole stands in the same place under every value a whole turn from the one naming
// it, so its bounds admit whichever of those namings falls between them; any other joint is admitted
// only at its own value. A bound pair the chain does not carry for a joint leaves that joint free.
std::optional<joint_vector> named_inside_bounds(const screw_chain &chain, const joint_vector &candidate)
{
    const joint_limits &bounds = chain.limits;

    joint_vector named = candidate;
    for(Eigen::Index joint = 0; joint < named.size(); ++joint)
    {
        if(joint >= bounds.lower_position.size() || joint >= bounds.upper_position.size())
            continue;

        const double turn                  = turns_whole(chain.space_screws[static_cast<std::size_t>(joint)]) ? 2.0 * std::numbers::pi : 0.0;
        const std::optional<double> inside = named_between(named[joint], bounds.lower_position[joint], bounds.upper_position[joint], turn);
        if(!inside.has_value())
            return std::nullopt;

        named[joint] = *inside;
    }

    return named;
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

expected<void, refusal> solved_inside_bounds(const screw_chain &chain, std::span<const screw_axis> body, const transform &desired, const joint_vector &j0,
                                             const solver_parameters &parameters, solve_step step, ik_result &answer)
{
    return answered_inside_bounds(chain, iterated(chain.home, body, desired, j0, parameters, step, answer), answer);
}

}
