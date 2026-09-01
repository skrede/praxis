#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_MOTION_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_MOTION_H

#include "praxis/manipulator/motion.h"

// Every declaration below matches a slot of motion_ops by name and by signature, so composing the
// aggregate is a plain address-of.
namespace praxis::manipulator {

expected<joint_vector, refusal> task_space_pose(const kinematics &solver, const transform &pose, const joint_vector &j0);
expected<joint_vector, refusal> task_space_screw(const rigid_motion::screw_ops &screw, const kinematics &solver, const transform &start_pose, const Eigen::Vector3d &w,
                                                 const Eigen::Vector3d &q, double theta_radians, double h, const joint_vector &j0);
expected<joint_vector, refusal> tool_frame_displace(const kinematics &solver, const transform &start_pose, const Eigen::Vector3d &offset, const rotation &orientation,
                                                    const joint_vector &j0);

}

#endif
