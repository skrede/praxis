#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_SLOTS_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_SLOTS_H

#include "praxis/trajectory/path.h"
#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/time_scaling.h"
#include "praxis/trajectory/pose_trajectory.h"

#include "praxis/extension/coverage.h"
#include "praxis/extension/slot_set.h"
#include "praxis/extension/descriptor.h"

#include <cstdint>

namespace praxis::trajectory {

// The enumerators are the aggregate's member names unqualified and in its declaration order: the
// capability is carried by the enumeration's own name, so a slot another extension spells the same
// way stays distinct. The trailing count enumerator names no slot; the slot set, the size assertions
// and the coverage functions all read it.
enum class time_scaling_slot : std::uint32_t
{
    cubic,
    quintic,
    trapezoidal,
    count,
};

enum class path_slot : std::uint32_t
{
    joint_straight_line,
    screw,
    decoupled,
    count,
};

enum class pose_trajectory_slot : std::uint32_t
{
    decoupled_pose_waypoints,
    screw_pose_waypoints,
    count,
};

enum class trajectory_slot : std::uint32_t
{
    joint_space_waypoints,
    count,
};

using time_scaling_slot_set    = basic_slot_set<time_scaling_slot>;
using path_slot_set            = basic_slot_set<path_slot>;
using pose_trajectory_slot_set = basic_slot_set<pose_trajectory_slot>;
using trajectory_slot_set      = basic_slot_set<trajectory_slot>;

// A view points into the value it was given, which must outlive it. A temporary argument would leave
// the view dangling at the end of the full expression, so that call is deleted rather than diagnosed
// at run time.
capability_view view_of(const time_scaling_ops &ops);
capability_view view_of(time_scaling_ops &&) = delete;
capability_view view_of(const path_ops &ops);
capability_view view_of(path_ops &&) = delete;
capability_view view_of(const pose_trajectory_ops &ops);
capability_view view_of(pose_trajectory_ops &&) = delete;
capability_view view_of(const trajectory_ops &ops);
capability_view view_of(trajectory_ops &&) = delete;

}

#endif
