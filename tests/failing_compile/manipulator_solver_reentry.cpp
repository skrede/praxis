#include "praxis/manipulator.h"

namespace praxis::manipulator::probe {

expected<void, refusal> solver_reentry(const forward_kinematics_ops &forward, const differential_kinematics_ops &differential, const screw_chain &chain, const transform &desired,
                                       const joint_vector &j0, const solver_parameters &parameters, ik_result &answer)
{
    return forward.inverse_kinematics(forward, differential, chain, desired, j0, parameters, answer);
}

}
