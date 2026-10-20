#include "praxis/manipulator/robot.h"

namespace praxis::manipulator::inert {

expected<transform, refusal> tool_pose_from_flange_pose(const transform &, const transform &)
{
    return unexpected(refusal::not_implemented);
}

expected<transform, refusal> flange_pose_from_tool_pose(const rigid_motion::frame_ops &, const transform &, const transform &)
{
    return unexpected(refusal::not_implemented);
}

expected<Eigen::Vector3d, refusal> position_from_pose(const transform &)
{
    return unexpected(refusal::not_implemented);
}

expected<rotation, refusal> orientation_from_pose(const transform &)
{
    return unexpected(refusal::not_implemented);
}

expected<joint_vector, refusal> ik_solve_pose(const kinematics &, const rigid_motion::frame_ops &, const transform &, const joint_vector &, const transform &)
{
    return unexpected(refusal::not_implemented);
}

expected<joint_vector, refusal> ik_solve_flange_pose(const kinematics &, const transform &, const joint_vector &)
{
    return unexpected(refusal::not_implemented);
}

}
