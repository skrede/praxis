#include "praxis/manipulator/kinematics.h"

#include <span>

namespace praxis::manipulator::inert {

expected<jacobian, refusal> space_jacobian(const rigid_motion::screw_ops &, std::span<const screw_axis>, const joint_vector &)
{
    return unexpected(refusal::not_implemented);
}

expected<jacobian, refusal> body_jacobian(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const transform &, std::span<const screw_axis>, const joint_vector &)
{
    return unexpected(refusal::not_implemented);
}

}
