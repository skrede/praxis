#include "praxis/manipulator/robot.h"

#include "praxis/manipulator/motion.h"
#include "praxis/manipulator/modeling.h"
#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/pose_readout.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <type_traits>
#include <initializer_list>

using namespace praxis;
using namespace praxis::manipulator;

static_assert(std::is_trivially_copyable_v<praxis::manipulator::robot_ops>);
static_assert(std::is_trivially_copyable_v<praxis::manipulator::motion_ops>);
static_assert(std::is_trivially_copyable_v<praxis::manipulator::modeling_ops>);
static_assert(std::is_trivially_copyable_v<praxis::rigid_motion::frame_ops>);
static_assert(std::is_trivially_copyable_v<praxis::rigid_motion::screw_ops>);

namespace {

Eigen::Vector3d constant_position(const transform &)
{
    return Eigen::Vector3d::Constant(4.0);
}

using pose_reading = pose_readout::pose_reading;
using frame_view   = pose_readout::frame_view;

robot_slot_set left_at_defaults(std::initializer_list<robot_slot> slots)
{
    robot_slot_set held;
    for(robot_slot slot : slots)
        held.set(slot);

    return held;
}

arm_snapshot reading(expected<Eigen::Vector3d, refusal> position, expected<rotation, refusal> orientation)
{
    return arm_snapshot{joint_vector(),
                        joint_limits{},
                        transform::Identity(),
                        transform::Identity(),
                        transform::Identity(),
                        position,
                        position,
                        orientation,
                        orientation,
                        recording_parameters{},
                        1.0,
                        false,
                        scheduler::task_counters{},
                        {},
                        praxis::unexpected(refusal::not_implemented),
                        praxis::unexpected(refusal::not_implemented),
                        jacobian_manipulability{praxis::unexpected(refusal::not_implemented), praxis::unexpected(refusal::not_implemented)},
                        jacobian_manipulability{praxis::unexpected(refusal::not_implemented), praxis::unexpected(refusal::not_implemented)},
                        {},
                        {},
                        nullptr,
                        nullptr,
                        {},
                        {}};
}

arm_snapshot valued()
{
    return reading(Eigen::Vector3d::Constant(4.0), rotation::Identity());
}

arm_snapshot refused()
{
    return reading(praxis::unexpected(refusal::not_implemented), praxis::unexpected(refusal::not_implemented));
}

// The readout is asked what it would render; a graphics context is reached by nothing here.
pose_reading answered(robot_slot_set inert, const arm_snapshot &seen, frame_view frame)
{
    const pose_readout readout(arm_publisher().reader(), rigid_motion::frame_ops{}, inert);

    return readout.reading_of(seen, frame);
}

}

TEST_CASE("the_pose_conversion_and_accessor_slots_default_to_identity_and_zero")
{
    robot_ops ops{};

    REQUIRE(ops.tool_pose_from_flange_pose != nullptr);
    REQUIRE(ops.flange_pose_from_tool_pose != nullptr);
    REQUIRE(ops.position_from_pose != nullptr);
    REQUIRE(ops.orientation_from_pose != nullptr);

    REQUIRE(is_approx_equal(ops.tool_pose_from_flange_pose(transform::Identity(), transform::Identity()), transform::Identity()));
    REQUIRE(is_approx_equal(ops.flange_pose_from_tool_pose(rigid_motion::frame_ops{}, transform::Identity(), transform::Identity()), transform::Identity()));
    REQUIRE(ops.position_from_pose(transform::Identity()).isZero(default_tolerance));
    REQUIRE(ops.orientation_from_pose(transform::Identity()).isApprox(rotation::Identity(), default_tolerance));
}

TEST_CASE("the_robot_inverse_kinematics_slots_default_to_refusing_rather_than_to_the_seed_configuration")
{
    robot_ops ops{};
    const kinematics solver{};
    const joint_vector j0 = joint_vector::Constant(3, 0.25);

    REQUIRE(ops.ik_solve_pose != nullptr);
    REQUIRE(ops.ik_solve_flange_pose != nullptr);

    const expected<joint_vector, refusal> from_tool   = ops.ik_solve_pose(solver, rigid_motion::frame_ops{}, transform::Identity(), j0, transform::Identity());
    const expected<joint_vector, refusal> from_flange = ops.ik_solve_flange_pose(solver, transform::Identity(), j0);

    REQUIRE_FALSE(from_tool.has_value());
    REQUIRE_FALSE(from_flange.has_value());
    CHECK(from_tool.error() == refusal::not_implemented);
    CHECK(from_flange.error() == refusal::not_implemented);
}

TEST_CASE("assigning_one_robot_slot_leaves_every_other_slot_on_its_default")
{
    robot_ops ops{.position_from_pose = &constant_position};

    REQUIRE(ops.position_from_pose == &constant_position);
    REQUIRE(ops.position_from_pose(transform::Identity()).isApprox(Eigen::Vector3d::Constant(4.0), default_tolerance));

    REQUIRE(ops.tool_pose_from_flange_pose == &inert::tool_pose_from_flange_pose);
    REQUIRE(ops.orientation_from_pose == &inert::orientation_from_pose);
    REQUIRE(ops.ik_solve_flange_pose == &inert::ik_solve_flange_pose);
}

TEST_CASE("a_reading_whose_accessors_were_left_at_their_defaults_is_answered_as_unbound_rather_than_as_the_value_they_fabricate")
{
    const robot_slot_set both = left_at_defaults({robot_slot::position_from_pose, robot_slot::orientation_from_pose});

    CHECK(answered(both, valued(), frame_view::tool) == pose_reading::both_unbound);
    CHECK(answered(both, valued(), frame_view::flange) == pose_reading::both_unbound);
}

TEST_CASE("a_reading_whose_accessors_are_bound_and_whose_snapshot_carries_a_refusal_is_answered_as_refused")
{
    CHECK(answered(robot_slot_set(), refused(), frame_view::tool) == pose_reading::refused);
    CHECK(answered(robot_slot_set(), refused(), frame_view::flange) == pose_reading::refused);
}

TEST_CASE("a_reading_whose_accessors_are_bound_and_whose_snapshot_carries_values_is_answered_as_a_value")
{
    CHECK(answered(robot_slot_set(), valued(), frame_view::tool) == pose_reading::value);
    CHECK(answered(robot_slot_set(), valued(), frame_view::flange) == pose_reading::value);
}

TEST_CASE("one_accessor_left_at_its_default_is_named_rather_than_reported_alongside_the_one_that_is_bound")
{
    CHECK(answered(left_at_defaults({robot_slot::position_from_pose}), valued(), frame_view::tool) == pose_reading::position_unbound);
    CHECK(answered(left_at_defaults({robot_slot::orientation_from_pose}), valued(), frame_view::tool) == pose_reading::orientation_unbound);
}

// A reading nothing produced cannot also have been declined, so the unbound answer is the earlier of
// the two even where the snapshot carries a refusal.
TEST_CASE("an_unbound_reading_is_answered_ahead_of_a_refused_one")
{
    const robot_slot_set both = left_at_defaults({robot_slot::position_from_pose, robot_slot::orientation_from_pose});

    CHECK(answered(both, refused(), frame_view::tool) == pose_reading::both_unbound);
}
