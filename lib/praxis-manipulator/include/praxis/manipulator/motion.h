#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_MOTION_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_MOTION_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/kinematics.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/types.h"

#include <Eigen/Core>

namespace praxis::manipulator::inert {

expected<joint_vector, refusal> task_space_pose(const kinematics &solver, const transform &pose, const joint_vector &j0);
expected<joint_vector, refusal> task_space_screw(const rigid_motion::screw_ops &screw, const kinematics &solver, const transform &start_pose, const Eigen::Vector3d &w,
                                                 const Eigen::Vector3d &q, double theta_radians, double h, const joint_vector &j0);
expected<joint_vector, refusal> tool_frame_displace(const kinematics &solver, const transform &start_pose, const Eigen::Vector3d &offset, const rotation &orientation,
                                                    const joint_vector &j0);

}

namespace praxis::manipulator {

// Declaration order is frozen: a designated initializer must name members in declaration order, so
// reordering a slot breaks every project that already composes this aggregate. Appending is safe.
struct motion_ops
{
    expected<joint_vector, refusal> (*task_space_pose)(const kinematics &solver, const transform &pose, const joint_vector &j0)           = &inert::task_space_pose;
    expected<joint_vector, refusal> (*task_space_screw)(const rigid_motion::screw_ops &screw, const kinematics &solver, const transform &start_pose, const Eigen::Vector3d &w,
                                                        const Eigen::Vector3d &q, double theta_radians, double h, const joint_vector &j0) = &inert::task_space_screw;
    expected<joint_vector, refusal> (*tool_frame_displace)(const kinematics &solver, const transform &start_pose, const Eigen::Vector3d &offset, const rotation &orientation,
                                                           const joint_vector &j0)                                                        = &inert::tool_frame_displace;
};

}

#endif
