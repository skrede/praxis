#include "chain_maps.h"
#include "opw_geometry.h"
#include "chain_translation.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/evaluation/tolerance.h"

#include <Eigen/QR>

#include <spdlog/spdlog.h>

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
                                         ik_result &answer)
{
    joint_vector theta             = j0;
    expected<twist, refusal> error = body_error(home, body, desired, theta);
    for(std::uint32_t i = 0; error && !within(*error, parameters) && i < parameters.max_iterations_per_attempt; ++i)
    {
        const expected<jacobian, refusal> jb = body_columns(body, theta);
        if(!jb)
            return unexpected(refusal::no_solution);

        const joint_vector previous = theta;
        theta += Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>(*jb).solve(*error);
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

// A closed form answers over the mechanism's geometry, which says nothing about where the joints are
// allowed to stand, so a candidate is kept only where the chain's bounds admit a naming of it and the
// chain's own forward map places that naming at the target. The admitted naming is what is answered.
std::optional<joint_vector> answered_naming(const rigid_motion::screw_ops &screw, const forward_kinematics_ops &forward, const screw_chain &chain, const transform &desired,
                                            const joint_vector &candidate)
{
    const std::optional<joint_vector> named = named_inside_bounds(chain, candidate);
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

// Lynch & Park, Modern Robotics, sec. 6.2.2; the step is eq. (6.6) in the body frame. A converged
// configuration is answered at its naming inside the chain's joint bounds.
expected<void, refusal> inverse_kinematics(const rigid_motion::screw_ops &, const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &chain,
                                           const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer)
{
    if(j0.size() != static_cast<Eigen::Index>(chain.joint_count()))
        return unexpected(refusal::unsupported_input);
    if(!is_admitted(chain) || !is_a_rigid_motion(desired) || !j0.allFinite())
        return unexpected(refusal::degenerate);

    const expected<std::vector<screw_axis>, refusal> body = to_body_screws(rigid_motion::baseline().screw, chain.home, chain.space_screws);
    if(!body)
        return unexpected(refusal::degenerate);

    const expected<joint_vector, refusal> converged = iterated(chain.home, *body, desired, j0, parameters, answer);
    if(!converged)
        return unexpected(converged.error());

    const std::optional<joint_vector> named = named_inside_bounds(chain, *converged);
    if(!named.has_value())
        return unexpected(refusal::no_solution);

    answer.solutions.push_back(*named);

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
