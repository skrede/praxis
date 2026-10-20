#include "praxis/manipulator.h"

namespace praxis::manipulator::probe {

expected<void, refusal> solver_ops_reentry(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward,
                                           const differential_kinematics_ops &differential, const optimization::solver_ops &solver, const screw_chain &chain, const transform &desired,
                                           const joint_vector &j0, const solver_parameters &parameters, ik_result &answer)
{
    return solver.inverse_kinematics(screw, frames, forward, differential, solver, chain, desired, j0, parameters, answer);
}

}
