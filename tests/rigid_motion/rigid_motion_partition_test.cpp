#include "praxis/rigid_motion/slots.h"

#include "praxis/compat/expected.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cstddef>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

// One pairing per slot of this extension: the slot's own enumerator beside whether the aggregate
// member it names returns through the refusal channel. Twenty-nine slots in total, which is the sum
// of the two enumerations this extension declares -- frame_slot and screw_slot -- each of which the
// descriptor table already holds to its own size. Nothing else compares an enumerator against the
// type of the member it stands for, so without these a slot changing sides is silent.
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

constexpr std::array frame_partition{
        pairing{static_cast<std::size_t>(frame_slot::euler_from_rotation_matrix), returns_refusal_v<decltype(frame_ops::euler_from_rotation_matrix)>},
        pairing{static_cast<std::size_t>(frame_slot::rotate_x), returns_refusal_v<decltype(frame_ops::rotate_x)>},
        pairing{static_cast<std::size_t>(frame_slot::rotate_y), returns_refusal_v<decltype(frame_ops::rotate_y)>},
        pairing{static_cast<std::size_t>(frame_slot::rotate_z), returns_refusal_v<decltype(frame_ops::rotate_z)>},
        pairing{static_cast<std::size_t>(frame_slot::rotation_matrix_from_frame_axes), returns_refusal_v<decltype(frame_ops::rotation_matrix_from_frame_axes)>},
        pairing{static_cast<std::size_t>(frame_slot::rotation_matrix_from_euler), returns_refusal_v<decltype(frame_ops::rotation_matrix_from_euler)>},
        pairing{static_cast<std::size_t>(frame_slot::rotation_matrix_from_axis_angle), returns_refusal_v<decltype(frame_ops::rotation_matrix_from_axis_angle)>},
        pairing{static_cast<std::size_t>(frame_slot::rotation_matrix_from_transform), returns_refusal_v<decltype(frame_ops::rotation_matrix_from_transform)>},
        pairing{static_cast<std::size_t>(frame_slot::transformation_matrix_from_position), returns_refusal_v<decltype(frame_ops::transformation_matrix_from_position)>},
        pairing{static_cast<std::size_t>(frame_slot::transformation_matrix_from_rotation), returns_refusal_v<decltype(frame_ops::transformation_matrix_from_rotation)>},
        pairing{static_cast<std::size_t>(frame_slot::transformation_matrix_from_rotation_position),
                returns_refusal_v<decltype(frame_ops::transformation_matrix_from_rotation_position)>},
        pairing{static_cast<std::size_t>(frame_slot::inverse), returns_refusal_v<decltype(frame_ops::inverse)>},
};

constexpr std::array screw_partition{
        pairing{static_cast<std::size_t>(screw_slot::skew_symmetric), returns_refusal_v<decltype(screw_ops::skew_symmetric)>},
        pairing{static_cast<std::size_t>(screw_slot::from_skew_symmetric), returns_refusal_v<decltype(screw_ops::from_skew_symmetric)>},
        pairing{static_cast<std::size_t>(screw_slot::adjoint_matrix_from_rotation_position), returns_refusal_v<decltype(screw_ops::adjoint_matrix_from_rotation_position)>},
        pairing{static_cast<std::size_t>(screw_slot::adjoint_matrix_from_transform), returns_refusal_v<decltype(screw_ops::adjoint_matrix_from_transform)>},
        pairing{static_cast<std::size_t>(screw_slot::adjoint_map), returns_refusal_v<decltype(screw_ops::adjoint_map)>},
        pairing{static_cast<std::size_t>(screw_slot::twist_from_angular_linear), returns_refusal_v<decltype(screw_ops::twist_from_angular_linear)>},
        pairing{static_cast<std::size_t>(screw_slot::twist_from_screw), returns_refusal_v<decltype(screw_ops::twist_from_screw)>},
        pairing{static_cast<std::size_t>(screw_slot::twist_matrix_from_angular_linear), returns_refusal_v<decltype(screw_ops::twist_matrix_from_angular_linear)>},
        pairing{static_cast<std::size_t>(screw_slot::twist_matrix_from_twist), returns_refusal_v<decltype(screw_ops::twist_matrix_from_twist)>},
        pairing{static_cast<std::size_t>(screw_slot::screw_axis_from_angular_linear), returns_refusal_v<decltype(screw_ops::screw_axis_from_angular_linear)>},
        pairing{static_cast<std::size_t>(screw_slot::screw_axis_from_point_direction_pitch), returns_refusal_v<decltype(screw_ops::screw_axis_from_point_direction_pitch)>},
        pairing{static_cast<std::size_t>(screw_slot::matrix_exponential_so3), returns_refusal_v<decltype(screw_ops::matrix_exponential_so3)>},
        pairing{static_cast<std::size_t>(screw_slot::matrix_exponential_se3), returns_refusal_v<decltype(screw_ops::matrix_exponential_se3)>},
        pairing{static_cast<std::size_t>(screw_slot::matrix_exponential_screw), returns_refusal_v<decltype(screw_ops::matrix_exponential_screw)>},
        pairing{static_cast<std::size_t>(screw_slot::matrix_logarithm_so3), returns_refusal_v<decltype(screw_ops::matrix_logarithm_so3)>},
        pairing{static_cast<std::size_t>(screw_slot::matrix_logarithm_se3_rp), returns_refusal_v<decltype(screw_ops::matrix_logarithm_se3_rp)>},
        pairing{static_cast<std::size_t>(screw_slot::matrix_logarithm_se3), returns_refusal_v<decltype(screw_ops::matrix_logarithm_se3)>},
};

static_assert(frame_partition.size() == static_cast<std::size_t>(frame_slot::count));
static_assert(screw_partition.size() == static_cast<std::size_t>(screw_slot::count));

static_assert(is_indexed_by_its_enumerator(frame_partition));
static_assert(is_indexed_by_its_enumerator(screw_partition));

// The frame capability is total by construction: every one of its operations is defined for any
// rotation, any transform and any vector it can be handed, including the inverse, which reads a
// rotation block and a position column and needs no membership of SE(3) to transpose the one and
// carry the other.
static_assert(!frame_partition[static_cast<std::size_t>(frame_slot::euler_from_rotation_matrix)].carries_the_channel);
static_assert(!frame_partition[static_cast<std::size_t>(frame_slot::rotate_x)].carries_the_channel);
static_assert(!frame_partition[static_cast<std::size_t>(frame_slot::rotate_y)].carries_the_channel);
static_assert(!frame_partition[static_cast<std::size_t>(frame_slot::rotate_z)].carries_the_channel);
static_assert(!frame_partition[static_cast<std::size_t>(frame_slot::rotation_matrix_from_frame_axes)].carries_the_channel);
static_assert(!frame_partition[static_cast<std::size_t>(frame_slot::rotation_matrix_from_euler)].carries_the_channel);
static_assert(!frame_partition[static_cast<std::size_t>(frame_slot::rotation_matrix_from_axis_angle)].carries_the_channel);
static_assert(!frame_partition[static_cast<std::size_t>(frame_slot::rotation_matrix_from_transform)].carries_the_channel);
static_assert(!frame_partition[static_cast<std::size_t>(frame_slot::transformation_matrix_from_position)].carries_the_channel);
static_assert(!frame_partition[static_cast<std::size_t>(frame_slot::transformation_matrix_from_rotation)].carries_the_channel);
static_assert(!frame_partition[static_cast<std::size_t>(frame_slot::transformation_matrix_from_rotation_position)].carries_the_channel);
static_assert(!frame_partition[static_cast<std::size_t>(frame_slot::inverse)].carries_the_channel);

// The nine total screw slots are arithmetic on what they are handed and answer for every value of
// it; the eight fallible ones each read a matrix as a member of a group, or a direction as naming a
// line, and have no answer when it is not one.
static_assert(!screw_partition[static_cast<std::size_t>(screw_slot::skew_symmetric)].carries_the_channel);
static_assert(!screw_partition[static_cast<std::size_t>(screw_slot::from_skew_symmetric)].carries_the_channel);
static_assert(screw_partition[static_cast<std::size_t>(screw_slot::adjoint_matrix_from_rotation_position)].carries_the_channel);
static_assert(screw_partition[static_cast<std::size_t>(screw_slot::adjoint_matrix_from_transform)].carries_the_channel);
static_assert(screw_partition[static_cast<std::size_t>(screw_slot::adjoint_map)].carries_the_channel);
static_assert(!screw_partition[static_cast<std::size_t>(screw_slot::twist_from_angular_linear)].carries_the_channel);
static_assert(screw_partition[static_cast<std::size_t>(screw_slot::twist_from_screw)].carries_the_channel);
static_assert(!screw_partition[static_cast<std::size_t>(screw_slot::twist_matrix_from_angular_linear)].carries_the_channel);
static_assert(!screw_partition[static_cast<std::size_t>(screw_slot::twist_matrix_from_twist)].carries_the_channel);
static_assert(!screw_partition[static_cast<std::size_t>(screw_slot::screw_axis_from_angular_linear)].carries_the_channel);
static_assert(screw_partition[static_cast<std::size_t>(screw_slot::screw_axis_from_point_direction_pitch)].carries_the_channel);
static_assert(!screw_partition[static_cast<std::size_t>(screw_slot::matrix_exponential_so3)].carries_the_channel);
static_assert(!screw_partition[static_cast<std::size_t>(screw_slot::matrix_exponential_se3)].carries_the_channel);
static_assert(!screw_partition[static_cast<std::size_t>(screw_slot::matrix_exponential_screw)].carries_the_channel);
static_assert(screw_partition[static_cast<std::size_t>(screw_slot::matrix_logarithm_so3)].carries_the_channel);
static_assert(screw_partition[static_cast<std::size_t>(screw_slot::matrix_logarithm_se3_rp)].carries_the_channel);
static_assert(screw_partition[static_cast<std::size_t>(screw_slot::matrix_logarithm_se3)].carries_the_channel);

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
    const std::size_t paired   = frame_partition.size() + screw_partition.size();
    const std::size_t fallible = counted(frame_partition) + counted(screw_partition);

    CHECK(paired == 29u);
    CHECK(fallible == 8u);
}
