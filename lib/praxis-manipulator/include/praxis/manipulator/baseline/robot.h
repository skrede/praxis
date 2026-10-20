#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_ROBOT_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_ROBOT_H

#include "praxis/manipulator/robot.h"

// Every declaration below matches a slot of robot_ops by name and by signature, so composing the
// aggregate is a plain address-of.
namespace praxis::manipulator {

expected<transform, refusal> tool_pose_from_flange_pose(const transform &flange_pose, const transform &tool_offset);
expected<transform, refusal> flange_pose_from_tool_pose(const rigid_motion::frame_ops &frames, const transform &tool_pose, const transform &tool_offset);

expected<Eigen::Vector3d, refusal> position_from_pose(const transform &pose);
expected<rotation, refusal> orientation_from_pose(const transform &pose);

expected<joint_vector, refusal> ik_solve_pose(const kinematics &solver, const rigid_motion::frame_ops &frames, const transform &tool_pose, const joint_vector &j0,
                                              const transform &tool_offset);
expected<joint_vector, refusal> ik_solve_flange_pose(const kinematics &solver, const transform &flange_pose, const joint_vector &j0);

}

#endif
