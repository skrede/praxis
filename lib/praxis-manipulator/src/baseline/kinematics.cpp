#include "chain_maps.h"
#include "iterated_solve.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include <Eigen/QR>

#include <spdlog/spdlog.h>

namespace praxis::manipulator {

namespace {

// Lynch & Park, Modern Robotics, sec. 6.2.2; the step is eq. (6.6) in the body frame.
expected<joint_vector, refusal> newton_step(const body_target &, const joint_vector &theta, const jacobian &jb, const twist &error)
{
    return joint_vector(theta + Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>(jb).solve(error));
}

}

expected<void, refusal> inverse_kinematics(const rigid_motion::screw_ops &, const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &chain,
                                           const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer)
{
    return iterated(chain, desired, j0, parameters, &newton_step, answer);
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
