#include "praxis/trajectory/capabilities.h"
#include "praxis/trajectory/baseline/path.h"
#include "praxis/trajectory/baseline/trajectory.h"
#include "praxis/trajectory/baseline/time_scaling.h"
#include "praxis/trajectory/baseline/pose_trajectory.h"

#include <array>

namespace praxis::trajectory {

namespace {

constexpr time_scaling_ops bound_time_scaling{
        .cubic       = &cubic,
        .quintic     = &quintic,
        .trapezoidal = &trapezoidal,
};

constexpr path_ops bound_path{
        .joint_straight_line = &joint_straight_line,
        .screw               = &screw,
        .decoupled           = &decoupled,
};

constexpr pose_trajectory_ops bound_pose_trajectory{
        .decoupled_pose_waypoints = &decoupled_pose_waypoints,
        .screw_pose_waypoints     = &screw_pose_waypoints,
};

constexpr trajectory_ops bound_trajectory{
        .joint_space_waypoints = &joint_space_waypoints,
};

}

capabilities baseline()
{
    return capabilities{
            .time_scaling    = bound_time_scaling,
            .path            = bound_path,
            .pose_trajectory = bound_pose_trajectory,
            .trajectory      = bound_trajectory,
    };
}

std::array<capability_view, 4> capability_views(const capabilities &c)
{
    return {view_of(c.time_scaling), view_of(c.path), view_of(c.pose_trajectory), view_of(c.trajectory)};
}

}
