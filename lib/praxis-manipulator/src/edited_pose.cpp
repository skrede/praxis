#include "praxis/manipulator/edited_pose.h"

#include "praxis/rigid_motion/angles.h"

#include <Eigen/Core>

namespace praxis::manipulator {

edited_pose::edited_pose()
        : order(axis_order::zyx)
        , standing(pose_standing::unset)
        , position(Eigen::Vector3f::Zero())
        , euler_degrees(Eigen::Vector3f::Zero())
{
}

transform pose_matrix(const edited_pose &edited, const rigid_motion::frame_ops &frames)
{
    const Eigen::Vector3d angles = edited.euler_degrees.cast<double>() * radians_per_degree;

    return frames.transformation_matrix_from_rotation_position(frames.rotation_matrix_from_euler(angles, edited.order), edited.position.cast<double>());
}

bool seed_from(edited_pose &edited, const arm_snapshot &seen, const rigid_motion::frame_ops &frames)
{
    if(!seen.tool_orientation || !seen.tool_position)
        return false;

    const Eigen::Vector3d angles = frames.euler_from_rotation_matrix(*seen.tool_orientation, edited.order);
    edited.position              = seen.tool_position->cast<float>();
    edited.euler_degrees         = (angles * degrees_per_radian).cast<float>();
    edited.standing              = pose_standing::held;

    return true;
}

void seed_unless_held(edited_pose &edited, const arm_snapshot &seen, const rigid_motion::frame_ops &frames)
{
    if(edited.standing == pose_standing::held || (edited.standing == pose_standing::provisional && !seen.tool_offset_known))
        return;

    if(seed_from(edited, seen, frames) && !seen.tool_offset_known)
        edited.standing = pose_standing::provisional;
}

}
