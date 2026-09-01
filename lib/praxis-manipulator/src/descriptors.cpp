#include "praxis/manipulator/capabilities.h"

#include <array>
#include <cstddef>

namespace praxis::manipulator {

namespace {

// An identical-code-folding linker can merge two byte-identical function bodies to one address. That
// can only report a slot as still holding its default when the supplied implementation is
// byte-identical to the inert one, in which case the report is correct.
constexpr std::array forward_kinematics_descriptors{
        slot_descriptor{"fk.forward_kinematics",
                        [](const void *value) -> bool { return static_cast<const forward_kinematics_ops *>(value)->forward_kinematics == &inert::forward_kinematics; }},
        slot_descriptor{"fk.body_forward_kinematics",
                        [](const void *value) -> bool { return static_cast<const forward_kinematics_ops *>(value)->body_forward_kinematics == &inert::body_forward_kinematics; }},
        slot_descriptor{"fk.body_screws_from_space",
                        [](const void *value) -> bool { return static_cast<const forward_kinematics_ops *>(value)->body_screws_from_space == &inert::body_screws_from_space; }},
};

constexpr std::array differential_kinematics_descriptors{
        slot_descriptor{"dk.space_jacobian",
                        [](const void *value) -> bool { return static_cast<const differential_kinematics_ops *>(value)->space_jacobian == &inert::space_jacobian; }},
        slot_descriptor{"dk.body_jacobian", [](const void *value) -> bool { return static_cast<const differential_kinematics_ops *>(value)->body_jacobian == &inert::body_jacobian; }},
};

constexpr std::array inverse_kinematics_descriptors{
        slot_descriptor{"ik.inverse_kinematics",
                        [](const void *value) -> bool { return static_cast<const inverse_kinematics_ops *>(value)->inverse_kinematics == &inert::inverse_kinematics; }},
        slot_descriptor{"ik.analytic_inverse_kinematics", [](const void *value) -> bool
                        { return static_cast<const inverse_kinematics_ops *>(value)->analytic_inverse_kinematics == &inert::analytic_inverse_kinematics; }},
};

constexpr std::array robot_descriptors{
        slot_descriptor{"robot.tool_pose_from_flange_pose",
                        [](const void *value) -> bool { return static_cast<const robot_ops *>(value)->tool_pose_from_flange_pose == &inert::tool_pose_from_flange_pose; }},
        slot_descriptor{"robot.flange_pose_from_tool_pose",
                        [](const void *value) -> bool { return static_cast<const robot_ops *>(value)->flange_pose_from_tool_pose == &inert::flange_pose_from_tool_pose; }},
        slot_descriptor{"robot.position_from_pose", [](const void *value) -> bool { return static_cast<const robot_ops *>(value)->position_from_pose == &inert::position_from_pose; }},
        slot_descriptor{"robot.orientation_from_pose",
                        [](const void *value) -> bool { return static_cast<const robot_ops *>(value)->orientation_from_pose == &inert::orientation_from_pose; }},
        slot_descriptor{"robot.ik_solve_pose", [](const void *value) -> bool { return static_cast<const robot_ops *>(value)->ik_solve_pose == &inert::ik_solve_pose; }},
        slot_descriptor{"robot.ik_solve_flange_pose",
                        [](const void *value) -> bool { return static_cast<const robot_ops *>(value)->ik_solve_flange_pose == &inert::ik_solve_flange_pose; }},
};

constexpr std::array motion_descriptors{
        slot_descriptor{"motion.task_space_pose", [](const void *value) -> bool { return static_cast<const motion_ops *>(value)->task_space_pose == &inert::task_space_pose; }},
        slot_descriptor{"motion.task_space_screw", [](const void *value) -> bool { return static_cast<const motion_ops *>(value)->task_space_screw == &inert::task_space_screw; }},
        slot_descriptor{"motion.tool_frame_displace",
                        [](const void *value) -> bool { return static_cast<const motion_ops *>(value)->tool_frame_displace == &inert::tool_frame_displace; }},
};

constexpr std::array trajectory_descriptors{
        slot_descriptor{"trajectory.task_space_waypoints",
                        [](const void *value) -> bool { return static_cast<const task_trajectory_ops *>(value)->task_space_waypoints == &inert::task_space_waypoints; }},
};

constexpr std::array modeling_descriptors{
        slot_descriptor{"modeling.build_chain", [](const void *value) -> bool { return static_cast<const modeling_ops *>(value)->build_chain == &inert::build_chain; }},
};

static_assert(forward_kinematics_descriptors.size() == static_cast<std::size_t>(forward_kinematics_slot::count));
static_assert(differential_kinematics_descriptors.size() == static_cast<std::size_t>(differential_kinematics_slot::count));
static_assert(inverse_kinematics_descriptors.size() == static_cast<std::size_t>(inverse_kinematics_slot::count));
static_assert(robot_descriptors.size() == static_cast<std::size_t>(robot_slot::count));
static_assert(motion_descriptors.size() == static_cast<std::size_t>(motion_slot::count));
static_assert(trajectory_descriptors.size() == static_cast<std::size_t>(task_trajectory_slot::count));
static_assert(modeling_descriptors.size() == static_cast<std::size_t>(modeling_slot::count));

constexpr capability_descriptors<robot_ops> described_robots{"manipulator", robot_descriptors};
constexpr capability_descriptors<motion_ops> described_motions{"manipulator", motion_descriptors};
constexpr capability_descriptors<modeling_ops> described_modelings{"manipulator", modeling_descriptors};
constexpr capability_descriptors<forward_kinematics_ops> described_forward_kinematics{"manipulator", forward_kinematics_descriptors};
constexpr capability_descriptors<differential_kinematics_ops> described_differential_kinematics{"manipulator", differential_kinematics_descriptors};
constexpr capability_descriptors<inverse_kinematics_ops> described_inverse_kinematics{"manipulator", inverse_kinematics_descriptors};
constexpr capability_descriptors<task_trajectory_ops> described_trajectories{"manipulator", trajectory_descriptors};

}

capability_view view_of(const forward_kinematics_ops &ops)
{
    return capability_view::of(ops, described_forward_kinematics);
}

capability_view view_of(const differential_kinematics_ops &ops)
{
    return capability_view::of(ops, described_differential_kinematics);
}

capability_view view_of(const inverse_kinematics_ops &ops)
{
    return capability_view::of(ops, described_inverse_kinematics);
}

capability_view view_of(const robot_ops &ops)
{
    return capability_view::of(ops, described_robots);
}

capability_view view_of(const motion_ops &ops)
{
    return capability_view::of(ops, described_motions);
}

capability_view view_of(const task_trajectory_ops &ops)
{
    return capability_view::of(ops, described_trajectories);
}

capability_view view_of(const modeling_ops &ops)
{
    return capability_view::of(ops, described_modelings);
}

}
