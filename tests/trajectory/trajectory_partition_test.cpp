#include "praxis/trajectory/slots.h"

#include "praxis/compat/expected.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cstddef>

using namespace praxis;
using namespace praxis::trajectory;

namespace {

// One pairing per slot of this extension: the slot's own enumerator beside whether the aggregate
// member it names returns through the refusal channel. Nine slots in total, which is the sum of
// the four enumerations this extension declares -- time_scaling_slot, path_slot,
// pose_trajectory_slot and trajectory_slot -- each of which the descriptor table already holds to
// its own size. Nothing else compares an enumerator against the type of the member it stands for,
// so without these a slot changing sides is silent.
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

constexpr std::array time_scaling_partition{
        pairing{static_cast<std::size_t>(time_scaling_slot::cubic), returns_refusal_v<decltype(time_scaling_ops::cubic)>},
        pairing{static_cast<std::size_t>(time_scaling_slot::quintic), returns_refusal_v<decltype(time_scaling_ops::quintic)>},
        pairing{static_cast<std::size_t>(time_scaling_slot::trapezoidal), returns_refusal_v<decltype(time_scaling_ops::trapezoidal)>},
};

constexpr std::array path_partition{
        pairing{static_cast<std::size_t>(path_slot::joint_straight_line), returns_refusal_v<decltype(path_ops::joint_straight_line)>},
        pairing{static_cast<std::size_t>(path_slot::screw), returns_refusal_v<decltype(path_ops::screw)>},
        pairing{static_cast<std::size_t>(path_slot::decoupled), returns_refusal_v<decltype(path_ops::decoupled)>},
};

constexpr std::array pose_trajectory_partition{
        pairing{static_cast<std::size_t>(pose_trajectory_slot::decoupled_pose_waypoints), returns_refusal_v<decltype(pose_trajectory_ops::decoupled_pose_waypoints)>},
        pairing{static_cast<std::size_t>(pose_trajectory_slot::screw_pose_waypoints), returns_refusal_v<decltype(pose_trajectory_ops::screw_pose_waypoints)>},
};

constexpr std::array trajectory_partition{
        pairing{static_cast<std::size_t>(trajectory_slot::joint_space_waypoints), returns_refusal_v<decltype(trajectory_ops::joint_space_waypoints)>},
};

static_assert(time_scaling_partition.size() == static_cast<std::size_t>(time_scaling_slot::count));
static_assert(path_partition.size() == static_cast<std::size_t>(path_slot::count));
static_assert(pose_trajectory_partition.size() == static_cast<std::size_t>(pose_trajectory_slot::count));
static_assert(trajectory_partition.size() == static_cast<std::size_t>(trajectory_slot::count));

static_assert(is_indexed_by_its_enumerator(time_scaling_partition));
static_assert(is_indexed_by_its_enumerator(path_partition));
static_assert(is_indexed_by_its_enumerator(pose_trajectory_partition));
static_assert(is_indexed_by_its_enumerator(trajectory_partition));

static_assert(time_scaling_partition[static_cast<std::size_t>(time_scaling_slot::cubic)].carries_the_channel);
static_assert(time_scaling_partition[static_cast<std::size_t>(time_scaling_slot::quintic)].carries_the_channel);
static_assert(time_scaling_partition[static_cast<std::size_t>(time_scaling_slot::trapezoidal)].carries_the_channel);

static_assert(path_partition[static_cast<std::size_t>(path_slot::joint_straight_line)].carries_the_channel);
static_assert(path_partition[static_cast<std::size_t>(path_slot::screw)].carries_the_channel);
static_assert(path_partition[static_cast<std::size_t>(path_slot::decoupled)].carries_the_channel);

static_assert(pose_trajectory_partition[static_cast<std::size_t>(pose_trajectory_slot::decoupled_pose_waypoints)].carries_the_channel);
static_assert(pose_trajectory_partition[static_cast<std::size_t>(pose_trajectory_slot::screw_pose_waypoints)].carries_the_channel);

static_assert(trajectory_partition[static_cast<std::size_t>(trajectory_slot::joint_space_waypoints)].carries_the_channel);

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
TEST_CASE("every_slot_of_this_extension_is_paired_with_its_enumerator_and_carries_the_channel")
{
    const std::size_t paired   = time_scaling_partition.size() + path_partition.size() + pose_trajectory_partition.size() + trajectory_partition.size();
    const std::size_t fallible = counted(time_scaling_partition) + counted(path_partition) + counted(pose_trajectory_partition) + counted(trajectory_partition);

    CHECK(paired == 9u);
    CHECK(fallible == 9u);
}
