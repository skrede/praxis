#include "iterated_solve.h"

#include "baseline/chain_maps.h"

#include <Eigen/QR>

#include <cstdint>
#include <utility>
#include <optional>

namespace praxis::manipulator {

namespace {

struct handed
{
    const rigid_motion::screw_ops &screw;
    const rigid_motion::frame_ops &frames;
    const forward_kinematics_ops &forward;
    const differential_kinematics_ops &differential;
    const screw_chain &chain;
    const transform &desired;
};

bool within(const twist &error, const solver_parameters &parameters)
{
    return error.head<3>().norm() <= parameters.orientation_tol && error.tail<3>().norm() <= parameters.position_tol;
}

void record(ik_result &answer, const joint_vector &theta, const twist &error, double step)
{
    answer.iterations.push_back(iteration_state{theta, error.head<3>().norm(), error.tail<3>().norm(), step, static_cast<std::uint32_t>(answer.iterations.size())});
}

// The angular half of V_b leads.
expected<twist, refusal> body_error(const handed &ops, const joint_vector &theta)
{
    const expected<transform, refusal> tsb = ops.forward.body_forward_kinematics(ops.screw, ops.frames, ops.chain.home, ops.chain.space_screws, theta);
    if(!tsb)
        return unexpected(tsb.error());
    const expected<std::pair<screw_axis, double>, refusal> logarithm = ops.screw.matrix_logarithm_se3(transform(ops.frames.inverse(*tsb) * ops.desired));
    if(!logarithm)
        return unexpected(logarithm.error());

    return twist(logarithm->first * logarithm->second);
}

std::optional<refusal> refused_at_entry(const handed &ops, const joint_vector &j0)
{
    if(j0.size() != static_cast<Eigen::Index>(ops.chain.joint_count()))
        return refusal::unsupported_input;
    if(!is_admitted(ops.chain) || !is_a_rigid_motion(ops.frames, ops.desired))
        return refusal::degenerate;
    if(!j0.allFinite())
        return refusal::no_solution;

    return std::nullopt;
}

// The stop test runs before each step and once after the last; the step is eq. (6.6) in the body frame.
expected<joint_vector, refusal> stepped(const handed &ops, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer)
{
    joint_vector theta             = j0;
    expected<twist, refusal> error = body_error(ops, theta);
    if(error)
        record(answer, theta, *error, 0.0);
    for(std::uint32_t i = 0; error && !within(*error, parameters) && i < parameters.max_iterations_per_attempt; ++i)
    {
        const expected<jacobian, refusal> jb = ops.differential.body_jacobian(ops.screw, ops.frames, ops.forward, ops.chain.home, ops.chain.space_screws, theta);
        if(!jb || jb->cols() != theta.size())
            return unexpected(refusal::no_solution);

        const joint_vector step = Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>(*jb).solve(*error);
        theta += step;
        error = body_error(ops, theta);
        if(error)
            record(answer, theta, *error, step.norm());
    }
    if(!error || !within(*error, parameters) || !theta.allFinite())
        return unexpected(refusal::no_solution);

    return theta;
}

}

expected<void, refusal> iterated(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward,
                                 const differential_kinematics_ops &differential, const screw_chain &chain, const transform &desired, const joint_vector &j0,
                                 const solver_parameters &parameters, ik_result &answer)
{
    const handed ops{screw, frames, forward, differential, chain, desired};
    if(const std::optional<refusal> refused = refused_at_entry(ops, j0))
        return unexpected(*refused);

    const expected<joint_vector, refusal> converged = stepped(ops, j0, parameters, answer);
    if(!converged)
        return unexpected(converged.error());

    answer.solutions.push_back(*converged);

    return {};
}

}
