#include "praxis/trajectory/slots.h"

#include <array>
#include <cstddef>

namespace praxis::trajectory {

namespace {

// An identical-code-folding linker can merge two byte-identical function bodies to one address. That
// can only report a slot as still holding its default when the supplied implementation is
// byte-identical to the inert one, in which case the report is correct.
constexpr std::array time_scaling_descriptors{
        slot_descriptor{"time_scaling.cubic", [](const void *value) -> bool { return static_cast<const time_scaling_ops *>(value)->cubic == &inert::cubic; }},
        slot_descriptor{"time_scaling.quintic", [](const void *value) -> bool { return static_cast<const time_scaling_ops *>(value)->quintic == &inert::quintic; }},
        slot_descriptor{"time_scaling.trapezoidal", [](const void *value) -> bool { return static_cast<const time_scaling_ops *>(value)->trapezoidal == &inert::trapezoidal; }},
};

constexpr std::array path_descriptors{
        slot_descriptor{"path.joint_straight_line", [](const void *value) -> bool { return static_cast<const path_ops *>(value)->joint_straight_line == &inert::joint_straight_line; }},
        slot_descriptor{"path.screw", [](const void *value) -> bool { return static_cast<const path_ops *>(value)->screw == &inert::screw; }},
        slot_descriptor{"path.decoupled", [](const void *value) -> bool { return static_cast<const path_ops *>(value)->decoupled == &inert::decoupled; }},
};

constexpr std::array pose_trajectory_descriptors{
        slot_descriptor{"pose_trajectory.decoupled_pose_waypoints",
                        [](const void *value) -> bool { return static_cast<const pose_trajectory_ops *>(value)->decoupled_pose_waypoints == &inert::decoupled_pose_waypoints; }},
        slot_descriptor{"pose_trajectory.screw_pose_waypoints",
                        [](const void *value) -> bool { return static_cast<const pose_trajectory_ops *>(value)->screw_pose_waypoints == &inert::screw_pose_waypoints; }},
};

constexpr std::array trajectory_descriptors{
        slot_descriptor{"trajectory.joint_space_waypoints",
                        [](const void *value) -> bool { return static_cast<const trajectory_ops *>(value)->joint_space_waypoints == &inert::joint_space_waypoints; }},
};

static_assert(time_scaling_descriptors.size() == static_cast<std::size_t>(time_scaling_slot::count));
static_assert(path_descriptors.size() == static_cast<std::size_t>(path_slot::count));
static_assert(pose_trajectory_descriptors.size() == static_cast<std::size_t>(pose_trajectory_slot::count));
static_assert(trajectory_descriptors.size() == static_cast<std::size_t>(trajectory_slot::count));

constexpr capability_descriptors<path_ops> described_paths{"trajectory", path_descriptors};
constexpr capability_descriptors<trajectory_ops> described_trajectories{"trajectory", trajectory_descriptors};
constexpr capability_descriptors<time_scaling_ops> described_time_scalings{"trajectory", time_scaling_descriptors};
constexpr capability_descriptors<pose_trajectory_ops> described_pose_trajectories{"trajectory", pose_trajectory_descriptors};

}

capability_view view_of(const time_scaling_ops &ops)
{
    return capability_view::of(ops, described_time_scalings);
}

capability_view view_of(const path_ops &ops)
{
    return capability_view::of(ops, described_paths);
}

capability_view view_of(const pose_trajectory_ops &ops)
{
    return capability_view::of(ops, described_pose_trajectories);
}

capability_view view_of(const trajectory_ops &ops)
{
    return capability_view::of(ops, described_trajectories);
}

}
