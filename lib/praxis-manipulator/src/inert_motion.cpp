#include "praxis/manipulator/motion.h"

namespace praxis::manipulator::inert {

expected<joint_vector, refusal> task_space_pose(const kinematics &, const transform &, const joint_vector &)
{
    return unexpected(refusal::not_implemented);
}

expected<joint_vector, refusal> task_space_screw(const rigid_motion::screw_ops &, const kinematics &, const transform &, const Eigen::Vector3d &, const Eigen::Vector3d &, double,
                                                 double, const joint_vector &)
{
    return unexpected(refusal::not_implemented);
}

expected<joint_vector, refusal> tool_frame_displace(const kinematics &, const transform &, const Eigen::Vector3d &, const rotation &, const joint_vector &)
{
    return unexpected(refusal::not_implemented);
}

}
