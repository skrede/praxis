#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/kinematics.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/types.h"

#include <Eigen/Core>

namespace praxis::manipulator::inert {

transform tool_pose_from_flange_pose(const transform &flange_pose, const transform &tool_offset);
transform flange_pose_from_tool_pose(const rigid_motion::frame_ops &frames, const transform &tool_pose, const transform &tool_offset);

Eigen::Vector3d position_from_pose(const transform &pose);
rotation orientation_from_pose(const transform &pose);

expected<joint_vector, refusal> ik_solve_pose(const kinematics &solver, const rigid_motion::frame_ops &frames, const transform &tool_pose, const joint_vector &j0,
                                              const transform &tool_offset);
expected<joint_vector, refusal> ik_solve_flange_pose(const kinematics &solver, const transform &flange_pose, const joint_vector &j0);

}

namespace praxis::manipulator {

// Declaration order is frozen: a designated initializer must name members in declaration order, so
// reordering a slot breaks every project that already composes this aggregate. Appending is safe.
struct robot_ops
{
    transform (*tool_pose_from_flange_pose)(const transform &flange_pose, const transform &tool_offset)                                      = &inert::tool_pose_from_flange_pose;
    transform (*flange_pose_from_tool_pose)(const rigid_motion::frame_ops &frames, const transform &tool_pose, const transform &tool_offset) = &inert::flange_pose_from_tool_pose;

    Eigen::Vector3d (*position_from_pose)(const transform &pose) = &inert::position_from_pose;
    rotation (*orientation_from_pose)(const transform &pose)     = &inert::orientation_from_pose;

    expected<joint_vector, refusal> (*ik_solve_pose)(const kinematics &solver, const rigid_motion::frame_ops &frames, const transform &tool_pose, const joint_vector &j0,
                                                     const transform &tool_offset)                                                          = &inert::ik_solve_pose;
    expected<joint_vector, refusal> (*ik_solve_flange_pose)(const kinematics &solver, const transform &flange_pose, const joint_vector &j0) = &inert::ik_solve_flange_pose;
};

}

#endif
