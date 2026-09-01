#include "praxis/manipulator/kinematics.h"

namespace praxis::manipulator::inert {

expected<void, refusal> inverse_kinematics(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const transform &, const joint_vector &,
                                           const solver_parameters &, ik_result &)
{
    return unexpected(refusal::not_implemented);
}

expected<void, refusal> analytic_inverse_kinematics(const forward_kinematics_ops &, const screw_chain &, const transform &, ik_result &)
{
    return unexpected(refusal::not_implemented);
}

}
