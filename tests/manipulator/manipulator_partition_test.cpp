#include "praxis/manipulator/slots.h"

#include "praxis/compat/expected.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cstddef>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

// One pairing per slot of this extension: the slot's own enumerator beside whether the aggregate
// member it names returns through the refusal channel. Seventeen slots in total, which is the sum of
// the seven enumerations this extension declares -- forward_kinematics_slot,
// differential_kinematics_slot, inverse_kinematics_slot, robot_slot, motion_slot,
// task_trajectory_slot and modeling_slot -- each of which the descriptor table already holds to its
// own size. Nothing else compares an enumerator against the type of the member it stands for, so
// without these a slot changing sides is silent.
struct pairing
{
    std::size_t slot;
    bool carries_the_channel;
};

constexpr bool is_indexed_by_its_enumerator(std::span<const pairing> partition)
{
    for(std::size_t i = 0; i < partition.size(); ++i)
        if(partition[i].slot != i)
            return false;

    return true;
}

constexpr std::array forward_kinematics_partition{
        pairing{static_cast<std::size_t>(forward_kinematics_slot::forward_kinematics), returns_refusal_v<decltype(forward_kinematics_ops::forward_kinematics)>},
        pairing{static_cast<std::size_t>(forward_kinematics_slot::body_forward_kinematics), returns_refusal_v<decltype(forward_kinematics_ops::body_forward_kinematics)>},
        pairing{static_cast<std::size_t>(forward_kinematics_slot::body_screws_from_space), returns_refusal_v<decltype(forward_kinematics_ops::body_screws_from_space)>},
};

constexpr std::array differential_kinematics_partition{
        pairing{static_cast<std::size_t>(differential_kinematics_slot::space_jacobian), returns_refusal_v<decltype(differential_kinematics_ops::space_jacobian)>},
        pairing{static_cast<std::size_t>(differential_kinematics_slot::body_jacobian), returns_refusal_v<decltype(differential_kinematics_ops::body_jacobian)>},
};

constexpr std::array inverse_kinematics_partition{
        pairing{static_cast<std::size_t>(inverse_kinematics_slot::inverse_kinematics), returns_refusal_v<decltype(inverse_kinematics_ops::inverse_kinematics)>},
        pairing{static_cast<std::size_t>(inverse_kinematics_slot::analytic_inverse_kinematics), returns_refusal_v<decltype(inverse_kinematics_ops::analytic_inverse_kinematics)>},
};

constexpr std::array robot_partition{
        pairing{static_cast<std::size_t>(robot_slot::tool_pose_from_flange_pose), returns_refusal_v<decltype(robot_ops::tool_pose_from_flange_pose)>},
        pairing{static_cast<std::size_t>(robot_slot::flange_pose_from_tool_pose), returns_refusal_v<decltype(robot_ops::flange_pose_from_tool_pose)>},
        pairing{static_cast<std::size_t>(robot_slot::position_from_pose), returns_refusal_v<decltype(robot_ops::position_from_pose)>},
        pairing{static_cast<std::size_t>(robot_slot::orientation_from_pose), returns_refusal_v<decltype(robot_ops::orientation_from_pose)>},
        pairing{static_cast<std::size_t>(robot_slot::ik_solve_pose), returns_refusal_v<decltype(robot_ops::ik_solve_pose)>},
        pairing{static_cast<std::size_t>(robot_slot::ik_solve_flange_pose), returns_refusal_v<decltype(robot_ops::ik_solve_flange_pose)>},
};

constexpr std::array motion_partition{
        pairing{static_cast<std::size_t>(motion_slot::task_space_pose), returns_refusal_v<decltype(motion_ops::task_space_pose)>},
        pairing{static_cast<std::size_t>(motion_slot::task_space_screw), returns_refusal_v<decltype(motion_ops::task_space_screw)>},
        pairing{static_cast<std::size_t>(motion_slot::tool_frame_displace), returns_refusal_v<decltype(motion_ops::tool_frame_displace)>},
};

constexpr std::array task_trajectory_partition{
        pairing{static_cast<std::size_t>(task_trajectory_slot::task_space_waypoints), returns_refusal_v<decltype(task_trajectory_ops::task_space_waypoints)>},
};

constexpr std::array modeling_partition{
        pairing{static_cast<std::size_t>(modeling_slot::build_chain), returns_refusal_v<decltype(modeling_ops::build_chain)>},
};

static_assert(forward_kinematics_partition.size() == static_cast<std::size_t>(forward_kinematics_slot::count));
static_assert(differential_kinematics_partition.size() == static_cast<std::size_t>(differential_kinematics_slot::count));
static_assert(inverse_kinematics_partition.size() == static_cast<std::size_t>(inverse_kinematics_slot::count));
static_assert(robot_partition.size() == static_cast<std::size_t>(robot_slot::count));
static_assert(motion_partition.size() == static_cast<std::size_t>(motion_slot::count));
static_assert(task_trajectory_partition.size() == static_cast<std::size_t>(task_trajectory_slot::count));
static_assert(modeling_partition.size() == static_cast<std::size_t>(modeling_slot::count));

static_assert(is_indexed_by_its_enumerator(forward_kinematics_partition));
static_assert(is_indexed_by_its_enumerator(differential_kinematics_partition));
static_assert(is_indexed_by_its_enumerator(inverse_kinematics_partition));
static_assert(is_indexed_by_its_enumerator(robot_partition));
static_assert(is_indexed_by_its_enumerator(motion_partition));
static_assert(is_indexed_by_its_enumerator(task_trajectory_partition));
static_assert(is_indexed_by_its_enumerator(modeling_partition));

static_assert(forward_kinematics_partition[static_cast<std::size_t>(forward_kinematics_slot::forward_kinematics)].carries_the_channel);
static_assert(forward_kinematics_partition[static_cast<std::size_t>(forward_kinematics_slot::body_forward_kinematics)].carries_the_channel);
static_assert(forward_kinematics_partition[static_cast<std::size_t>(forward_kinematics_slot::body_screws_from_space)].carries_the_channel);

static_assert(differential_kinematics_partition[static_cast<std::size_t>(differential_kinematics_slot::space_jacobian)].carries_the_channel);
static_assert(differential_kinematics_partition[static_cast<std::size_t>(differential_kinematics_slot::body_jacobian)].carries_the_channel);

static_assert(inverse_kinematics_partition[static_cast<std::size_t>(inverse_kinematics_slot::inverse_kinematics)].carries_the_channel);
static_assert(inverse_kinematics_partition[static_cast<std::size_t>(inverse_kinematics_slot::analytic_inverse_kinematics)].carries_the_channel);

// The four pose conversions and accessors are total: each answers for every rigid motion it can be
// handed, so it has nothing to refuse and no channel to refuse through.
static_assert(!robot_partition[static_cast<std::size_t>(robot_slot::tool_pose_from_flange_pose)].carries_the_channel);
static_assert(!robot_partition[static_cast<std::size_t>(robot_slot::flange_pose_from_tool_pose)].carries_the_channel);
static_assert(!robot_partition[static_cast<std::size_t>(robot_slot::position_from_pose)].carries_the_channel);
static_assert(!robot_partition[static_cast<std::size_t>(robot_slot::orientation_from_pose)].carries_the_channel);
static_assert(robot_partition[static_cast<std::size_t>(robot_slot::ik_solve_pose)].carries_the_channel);
static_assert(robot_partition[static_cast<std::size_t>(robot_slot::ik_solve_flange_pose)].carries_the_channel);

static_assert(motion_partition[static_cast<std::size_t>(motion_slot::task_space_pose)].carries_the_channel);
static_assert(motion_partition[static_cast<std::size_t>(motion_slot::task_space_screw)].carries_the_channel);
static_assert(motion_partition[static_cast<std::size_t>(motion_slot::tool_frame_displace)].carries_the_channel);

static_assert(task_trajectory_partition[static_cast<std::size_t>(task_trajectory_slot::task_space_waypoints)].carries_the_channel);

static_assert(modeling_partition[static_cast<std::size_t>(modeling_slot::build_chain)].carries_the_channel);

constexpr std::size_t counted(std::span<const pairing> partition)
{
    std::size_t fallible = 0;
    for(const pairing &paired : partition)
        fallible += paired.carries_the_channel ? 1u : 0u;

    return fallible;
}

}

// The assertions above are the gate and they have already run by the time this case is entered; it
// reports the totals so that a run names the extension's share rather than passing in silence.
TEST_CASE("every_slot_of_this_extension_is_paired_with_its_enumerator_on_the_side_the_partition_puts_it")
{
    const std::size_t paired = forward_kinematics_partition.size() + differential_kinematics_partition.size() + inverse_kinematics_partition.size() + robot_partition.size() +
            motion_partition.size() + task_trajectory_partition.size() + modeling_partition.size();
    const std::size_t fallible = counted(forward_kinematics_partition) + counted(differential_kinematics_partition) + counted(inverse_kinematics_partition) + counted(robot_partition) +
            counted(motion_partition) + counted(task_trajectory_partition) + counted(modeling_partition);

    CHECK(paired == 18u);
    CHECK(fallible == 14u);
}
