#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SLOTS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SLOTS_H

#include "praxis/manipulator/robot.h"
#include "praxis/manipulator/motion.h"
#include "praxis/manipulator/modeling.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/task_trajectory.h"

#include "praxis/extension/coverage.h"
#include "praxis/extension/slot_set.h"
#include "praxis/extension/descriptor.h"

#include <cstdint>

namespace praxis::manipulator {

// The enumerators are the aggregate's member names unqualified and in its declaration order: the
// capability is carried by the enumeration's own name, so a slot another extension spells the same
// way stays distinct. The trailing count enumerator names no slot; the slot set, the size assertions
// and the coverage functions all read it.
enum class forward_kinematics_slot : std::uint32_t
{
    forward_kinematics,
    body_forward_kinematics,
    body_screws_from_space,
    count,
};

enum class differential_kinematics_slot : std::uint32_t
{
    space_jacobian,
    body_jacobian,
    count,
};

enum class inverse_kinematics_slot : std::uint32_t
{
    inverse_kinematics,
    analytic_inverse_kinematics,
    count,
};

enum class robot_slot : std::uint32_t
{
    tool_pose_from_flange_pose,
    flange_pose_from_tool_pose,
    position_from_pose,
    orientation_from_pose,
    ik_solve_pose,
    ik_solve_flange_pose,
    count,
};

enum class motion_slot : std::uint32_t
{
    task_space_pose,
    task_space_screw,
    tool_frame_displace,
    count,
};

enum class task_trajectory_slot : std::uint32_t
{
    task_space_waypoints,
    count,
};

enum class modeling_slot : std::uint32_t
{
    build_chain,
    count,
};

using forward_kinematics_slot_set      = basic_slot_set<forward_kinematics_slot>;
using differential_kinematics_slot_set = basic_slot_set<differential_kinematics_slot>;
using inverse_kinematics_slot_set      = basic_slot_set<inverse_kinematics_slot>;
using robot_slot_set                   = basic_slot_set<robot_slot>;
using motion_slot_set                  = basic_slot_set<motion_slot>;
using task_trajectory_slot_set         = basic_slot_set<task_trajectory_slot>;
using modeling_slot_set                = basic_slot_set<modeling_slot>;

// A view points into the value it was given, which must outlive it. A temporary argument would leave
// the view dangling at the end of the full expression, so that call is deleted rather than diagnosed
// at run time.
capability_view view_of(const forward_kinematics_ops &ops);
capability_view view_of(forward_kinematics_ops &&) = delete;
capability_view view_of(const differential_kinematics_ops &ops);
capability_view view_of(differential_kinematics_ops &&) = delete;
capability_view view_of(const inverse_kinematics_ops &ops);
capability_view view_of(inverse_kinematics_ops &&) = delete;
capability_view view_of(const robot_ops &ops);
capability_view view_of(robot_ops &&) = delete;
capability_view view_of(const motion_ops &ops);
capability_view view_of(motion_ops &&) = delete;
capability_view view_of(const task_trajectory_ops &ops);
capability_view view_of(task_trajectory_ops &&) = delete;
capability_view view_of(const modeling_ops &ops);
capability_view view_of(modeling_ops &&) = delete;

}

#endif
