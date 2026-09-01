#include "praxis/manipulator.h"

#include "praxis/rigid_motion/capabilities.h"

#include <span>
#include <vector>
#include <utility>

namespace robotics_course {

// Each function is of what its own equation consumes and of nothing else.
praxis::expected<praxis::transform, praxis::refusal> forward_kinematics(const praxis::transform &m, std::span<const praxis::screw_axis>, const praxis::manipulator::joint_vector &)
{
    return m;
}

praxis::expected<praxis::manipulator::jacobian, praxis::refusal> space_jacobian(std::span<const praxis::screw_axis> space_screws, const praxis::manipulator::joint_vector &)
{
    return praxis::manipulator::jacobian::Zero(6, static_cast<Eigen::Index>(space_screws.size()));
}

// The solve reads forward kinematics off the aggregate it is handed rather than repeating it, so it
// is whichever one the composition bound. The iteration sequence it records is offered for inspection
// afterwards without the solve owning a buffer of its own.
praxis::expected<void, praxis::refusal> inverse_kinematics(const praxis::manipulator::forward_kinematics_ops &forward, const praxis::manipulator::differential_kinematics_ops &,
                                                           const praxis::manipulator::screw_chain &chain, const praxis::transform &, const praxis::manipulator::joint_vector &j0,
                                                           const praxis::manipulator::solver_parameters &parameters, praxis::manipulator::ik_result &answer)
{
    const praxis::expected<praxis::transform, praxis::refusal> reached = forward.forward_kinematics(chain.home, chain.space_screws, j0);
    if(!reached)
        return praxis::unexpected(reached.error());

    answer.solutions.push_back(j0);
    answer.iterations.push_back(praxis::manipulator::iteration_state{j0, 0.0, (*reached)(2, 3), parameters.position_tol, 0u});

    return {};
}

// Composing the aggregates is what proves the functions match the seam: a signature that does not
// match the slot it is assigned to does not compile. A slot no initializer names keeps its inert
// default.
praxis::expected<praxis::manipulator::kinematics, praxis::refusal> make_solver(praxis::manipulator::screw_chain chain)
{
    return praxis::manipulator::kinematics::compose(std::move(chain), praxis::manipulator::forward_kinematics_ops{.forward_kinematics = &forward_kinematics},
                                                    praxis::manipulator::differential_kinematics_ops{.space_jacobian = &space_jacobian},
                                                    praxis::manipulator::inverse_kinematics_ops{.inverse_kinematics = &inverse_kinematics}, praxis::rigid_motion::baseline().screw,
                                                    praxis::rigid_motion::baseline().frame);
}

}
