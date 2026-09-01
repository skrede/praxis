#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/robot.h"
#include "praxis/manipulator/baseline/motion.h"
#include "praxis/manipulator/baseline/modeling.h"
#include "praxis/manipulator/baseline/kinematics.h"
#include "praxis/manipulator/baseline/task_trajectory.h"

#include <array>

namespace praxis::manipulator {

namespace {

constexpr forward_kinematics_ops bound_forward_kinematics{
        .forward_kinematics      = &forward_kinematics,
        .body_forward_kinematics = &body_forward_kinematics,
        .body_screws_from_space  = &body_screws_from_space,
};

constexpr differential_kinematics_ops bound_differential_kinematics{
        .space_jacobian = &space_jacobian,
        .body_jacobian  = &body_jacobian,
};

constexpr inverse_kinematics_ops bound_inverse_kinematics{
        .inverse_kinematics          = &inverse_kinematics,
        .analytic_inverse_kinematics = &analytic_inverse_kinematics,
};

constexpr robot_ops bound_robot{
        .tool_pose_from_flange_pose = &tool_pose_from_flange_pose,
        .flange_pose_from_tool_pose = &flange_pose_from_tool_pose,
        .position_from_pose         = &position_from_pose,
        .orientation_from_pose      = &orientation_from_pose,
        .ik_solve_pose              = &ik_solve_pose,
        .ik_solve_flange_pose       = &ik_solve_flange_pose,
};

constexpr motion_ops bound_motion{
        .task_space_pose     = &task_space_pose,
        .task_space_screw    = &task_space_screw,
        .tool_frame_displace = &tool_frame_displace,
};

constexpr modeling_ops bound_modeling{
        .build_chain = &build_chain,
};

constexpr task_trajectory_ops bound_trajectory{
        .task_space_waypoints = &task_space_waypoints,
};

}

capabilities baseline()
{
    return capabilities{
            .fk         = bound_forward_kinematics,
            .dk         = bound_differential_kinematics,
            .ik         = bound_inverse_kinematics,
            .robot      = bound_robot,
            .motion     = bound_motion,
            .modeling   = bound_modeling,
            .trajectory = bound_trajectory,
    };
}

std::array<capability_view, 7> capability_views(const capabilities &c)
{
    return {view_of(c.fk), view_of(c.dk), view_of(c.ik), view_of(c.robot), view_of(c.motion), view_of(c.modeling), view_of(c.trajectory)};
}

}
