#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_EDITED_POSE_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_EDITED_POSE_H

#include "praxis/manipulator/arm_snapshot.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/axis_order.h"

#include <Eigen/Core>

namespace praxis::manipulator {

// The angles are degrees and they are Euler angles about `order`; the position and the pose they
// compose are expressed in the space frame, which is the frame a published tool pose arrives in.
// The angles are read in `order` where they are read, so moving `order` reinterprets the angles
// already held rather than re-deriving them, and the pose the value stands for moves with it.
struct edited_pose
{
    edited_pose();

    axis_order order;
    Eigen::Vector3f position;
    Eigen::Vector3f euler_degrees;
};

transform pose_matrix(const edited_pose &edited, const rigid_motion::frame_ops &frames);

// False where the snapshot carries no tool pose, leaving `edited` as it stands.
bool seed_from(edited_pose &edited, const arm_snapshot &seen, const rigid_motion::frame_ops &frames);

}

#endif
