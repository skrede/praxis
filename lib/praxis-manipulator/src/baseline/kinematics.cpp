#include "chain_maps.h"
#include "opw_geometry.h"
#include "chain_translation.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include <cartan/serial/fk.h>
#include <cartan/serial/ik/solver/lm.h>

#include <spdlog/spdlog.h>

#include <cmath>
#include <limits>
#include <vector>
#include <numbers>
#include <optional>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// The dependency takes its budgets as a signed count, so one beyond that range is clamped rather than
// wrapped: an unclamped conversion turns a request for more effort into a negative limit meaning none.
// The loop below is the whole attempt, so the one budget praxis publishes bounds both of the
// dependency's.
cartan::convergence_criteria<double> to_criteria(const solver_parameters &parameters)
{
    constexpr std::uint32_t ceiling = static_cast<std::uint32_t>(std::numeric_limits<int>::max());
    const int limit                 = static_cast<int>(std::min(parameters.max_iterations_per_attempt, ceiling));

    return {parameters.position_tol, parameters.orientation_tol, limit, limit};
}

// V_b = log(T_sb(theta)^-1 T_sd): Lynch & Park, Modern Robotics, eq. (6.4). The angular half leads.
// The solve policy reports only the norm of the whole twist, so the two halves are taken here.
twist body_error(const chain_type &chain, const cartan::se3<double> &target, const joint_vector &theta)
{
    const auto reached = cartan::forward_kinematics(chain, theta);
    if(!reached)
        return twist::Zero();

    return (reached->end_effector.inverse() * target).log();
}

void record(ik_result &answer, const joint_vector &solution, const twist &error, const joint_vector &previous)
{
    answer.iterations.push_back(
            iteration_state{solution, error.head<3>().norm(), error.tail<3>().norm(), (solution - previous).norm(), static_cast<std::uint32_t>(answer.iterations.size())});
}

// A revolute joint stands in the same place under every value a whole turn from the one naming it, so
// a bound pair admits a value at whichever of those namings falls between them, and the naming nearest
// the one asked about is taken. A bound pair the chain does not carry for a joint leaves that joint
// free, as it does where the chain is translated for the solver library.
std::optional<joint_vector> named_inside_bounds(const joint_limits &bounds, const joint_vector &candidate)
{
    constexpr double turn = 2.0 * std::numbers::pi;

    joint_vector named = candidate;
    for(Eigen::Index joint = 0; joint < named.size(); ++joint)
    {
        if(joint >= bounds.lower_position.size() || joint >= bounds.upper_position.size())
            continue;

        const double fewest = std::ceil((bounds.lower_position[joint] - named[joint]) / turn);
        const double most   = std::floor((bounds.upper_position[joint] - named[joint]) / turn);
        if(fewest > most)
            return std::nullopt;

        named[joint] += turn * std::clamp(0.0, fewest, most);
    }

    return named;
}

// A closed form answers over the mechanism's geometry, which says nothing about where the joints are
// allowed to stand, so a candidate is kept only where the chain's bounds admit a naming of it and the
// chain's own forward map places that naming at the target. The admitted naming is what is answered.
std::optional<joint_vector> answered_naming(const rigid_motion::screw_ops &screw, const forward_kinematics_ops &forward, const screw_chain &chain, const transform &desired,
                                            const joint_vector &candidate)
{
    const std::optional<joint_vector> named = named_inside_bounds(chain.limits, candidate);
    if(!named.has_value())
        return std::nullopt;

    const expected<transform, refusal> reached = forward.forward_kinematics(screw, chain.home, chain.space_screws, *named);
    if(!reached)
        return std::nullopt;

    const auto held   = cartan::se3<double>::from_matrix(*reached);
    const auto target = cartan::se3<double>::from_matrix(desired);
    if(!held.has_value() || !target.has_value())
        return std::nullopt;

    const twist residual = (held.value().inverse() * target.value()).log();
    if(residual.head<3>().norm() > cartan::default_verification_tolerance_v<double>.orientation() ||
       residual.tail<3>().norm() > cartan::default_verification_tolerance_v<double>.position())
        return std::nullopt;

    return named;
}

// The branches the chain's own forward map places at the target, of the up to eight the closed form
// names; a target every one of them misses has no answer rather than leaving the chain refused.
expected<void, refusal> kept_branches(const rigid_motion::screw_ops &screw, const forward_kinematics_ops &forward, const screw_chain &chain, const transform &desired,
                                      const cartan::analytical_result<double, 6, 8> &branches, ik_result &answer)
{
    for(const Eigen::Vector<double, 6> &branch : branches)
        if(const std::optional<joint_vector> named = answered_naming(screw, forward, chain, desired, joint_vector(branch)); named.has_value())
            answer.solutions.push_back(*named);
    if(answer.solutions.empty())
        return unexpected(refusal::no_solution);

    return {};
}

// The parameters the chain's own geometry yields, kept only where the reconstruction against that
// chain's forward map holds.
expected<cartan::opw_parameters<double>, refusal> admitted_geometry(const rigid_motion::screw_ops &screw, const forward_kinematics_ops &forward, const screw_chain &chain)
{
    const expected<cartan::opw_parameters<double>, refusal> geometry = to_opw_parameters(chain);
    const expected<void, refusal> reconstructed = geometry ? agrees_with_chain(screw, forward, chain, *geometry) : expected<void, refusal>(unexpected(geometry.error()));
    if(reconstructed)
        return geometry;

    spdlog::error("praxis: 'ik.analytic_inverse_kinematics' was given a chain of {} joints it cannot take the ortho-parallel decomposition of, so no closed form is solved over it",
                  chain.joint_count());

    return unexpected(reconstructed.error());
}

}

// The iterate sequence is taken from the solve policy one work unit at a time. The solve is the
// solver library's, so neither the handed screw operations nor either kinematics aggregate enters it.
expected<void, refusal> inverse_kinematics(const rigid_motion::screw_ops &, const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &chain,
                                           const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer)
{
    const std::optional<chain_type> solved_over = to_cartan_chain(chain);
    const auto target                           = cartan::se3<double>::from_matrix(desired);
    if(!solved_over.has_value() || !target.has_value())
        return unexpected(refusal::degenerate);

    cartan::lm<chain_type> policy;
    policy.setup(solved_over.value(), target.value(), j0, to_criteria(parameters));
    for(joint_vector previous = j0; policy.status() == cartan::ik_status::running; previous = policy.solution())
    {
        policy.step(solved_over.value(), 1);
        record(answer, policy.solution(), body_error(solved_over.value(), target.value(), policy.solution()), previous);
    }

    const joint_vector solution = policy.solution();
    if(!policy.converged() || solution.hasNaN())
        return unexpected(refusal::no_solution);

    answer.solutions.push_back(solution);

    return {};
}

// The closed form for an ortho-parallel basis with a spherical wrist: Brandstotter, Angerer &
// Hofbaur (2014). Every branch it names is answered for, and the answer carries no iterates because
// none were taken.
expected<void, refusal> analytic_inverse_kinematics(const rigid_motion::screw_ops &screw, const forward_kinematics_ops &forward, const screw_chain &chain, const transform &desired,
                                                    ik_result &answer)
{
    const expected<cartan::opw_parameters<double>, refusal> geometry = admitted_geometry(screw, forward, chain);
    if(!geometry)
        return unexpected(geometry.error());

    const std::optional<chain_type> solved_over = to_cartan_chain(chain);
    const auto target                           = cartan::se3<double>::from_matrix(desired);
    if(!solved_over.has_value() || !target.has_value())
        return unexpected(refusal::degenerate);

    const auto solver = cartan::opw_6r_solver<chain_type>::make(solved_over.value(), geometry.value());
    if(!solver)
        return unexpected(refusal_from(solver.error().reason));

    const auto branches = solver->solve(target.value());
    if(!branches)
        return unexpected(refusal_from(branches.error().reason));

    return kept_branches(screw, forward, chain, desired, branches.value(), answer);
}

expected<kinematics, refusal> make_kinematics(const screw_chain &chain, forward_kinematics_ops forward, differential_kinematics_ops differential, inverse_kinematics_ops inverse,
                                              const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames)
{
    if(!is_admitted(chain))
    {
        spdlog::error("praxis: 'manipulator.make_kinematics' was given a chain of {} joints that is empty or carries a value that is not finite, a home pose that is not a "
                      "rigid motion or a screw axis that is not of unit length, so no solver is composed",
                      chain.joint_count());

        return unexpected(refusal::degenerate);
    }

    expected<kinematics, refusal> composed = kinematics::compose(chain, forward, differential, inverse, screw, frames);
    if(!composed)
        spdlog::error("praxis: 'manipulator.make_kinematics' was given a chain of {} joints whose limits carry {}, {}, {} and {} entries, so no solver is composed", chain.joint_count(),
                      chain.limits.velocity.size(), chain.limits.acceleration.size(), chain.limits.lower_position.size(), chain.limits.upper_position.size());

    return composed;
}

}
