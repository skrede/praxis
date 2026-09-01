#include "praxis/manipulator/kinematics.h"

#include <span>
#include <vector>

namespace praxis::manipulator::inert {

expected<transform, refusal> forward_kinematics(const transform &, std::span<const screw_axis>, const joint_vector &)
{
    return unexpected(refusal::not_implemented);
}

expected<transform, refusal> body_forward_kinematics(const rigid_motion::frame_ops &, const transform &, std::span<const screw_axis>, const joint_vector &)
{
    return unexpected(refusal::not_implemented);
}

expected<std::vector<screw_axis>, refusal> body_screws_from_space(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const transform &, std::span<const screw_axis>)
{
    return unexpected(refusal::not_implemented);
}

}
