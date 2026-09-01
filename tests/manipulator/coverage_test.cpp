#include "praxis/manipulator.h"

#include "praxis/rigid_motion/screw.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <span>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

using namespace praxis;
using namespace praxis::manipulator;

static_assert(modeling_slot_set().empty());
static_assert(modeling_slot_set().set(modeling_slot::build_chain).contains(modeling_slot::build_chain));

namespace {

expected<joint_vector, refusal> zeroed_pose(const kinematics &, const transform &, const joint_vector &j0)
{
    return joint_vector(joint_vector::Zero(j0.size()));
}

// Every capability this module publishes, held where the views into them outlive the reports the
// cases below assemble.
const forward_kinematics_ops unbound_forward_kinematics{};
const differential_kinematics_ops unbound_differential_kinematics{};
const inverse_kinematics_ops unbound_inverse_kinematics{};
const robot_ops unbound_robot{};
const motion_ops unbound_motion{};
const task_trajectory_ops unbound_trajectory{};
const modeling_ops unbound_modeling{};

std::array<capability_view, 7> unbound_views()
{
    return {view_of(unbound_forward_kinematics),
            view_of(unbound_differential_kinematics),
            view_of(unbound_inverse_kinematics),
            view_of(unbound_robot),
            view_of(unbound_motion),
            view_of(unbound_trajectory),
            view_of(unbound_modeling)};
}

std::size_t slot_count(std::span<const capability_view> views)
{
    std::size_t described = 0;
    for(const capability_view &view : views)
    {
        described += view.slots().size();
    }
    return described;
}

}

TEST_CASE("a_value_initialized_capability_holds_the_named_inert_function_in_every_slot_it_defaults")
{
    CHECK(unbound_forward_kinematics.forward_kinematics == &inert::forward_kinematics);
    CHECK(unbound_forward_kinematics.body_forward_kinematics == &inert::body_forward_kinematics);
    CHECK(unbound_robot.tool_pose_from_flange_pose == &inert::tool_pose_from_flange_pose);
    CHECK(unbound_motion.task_space_pose == &inert::task_space_pose);
    CHECK(unbound_trajectory.task_space_waypoints == &inert::task_space_waypoints);
    CHECK(unbound_modeling.build_chain == &inert::build_chain);
    CHECK(rigid_motion::screw_ops{}.from_skew_symmetric == &rigid_motion::inert::from_skew_symmetric);
}

TEST_CASE("the_descriptor_table_covers_every_slot_once_under_a_unique_non_empty_name")
{
    const std::array<capability_view, 7> reported = unbound_views();
    std::set<std::string_view> names;

    for(const capability_view &view : reported)
    {
        for(const slot_descriptor &descriptor : view.slots())
        {
            REQUIRE_FALSE(descriptor.name.empty());
            REQUIRE(names.insert(descriptor.name).second);
        }
    }
    REQUIRE(names.size() == slot_count(reported));
}

TEST_CASE("a_value_initialized_capability_reports_every_slot_as_holding_its_default")
{
    const std::array<capability_view, 7> reported = unbound_views();

    REQUIRE(count_defaults(reported) == slot_count(reported));

    for(const capability_view &view : reported)
    {
        for(std::size_t index = 0; index < view.slots().size(); ++index)
        {
            REQUIRE(holds_default(view, index));
        }
    }
}

TEST_CASE("overriding_one_slot_leaves_every_other_slot_reported_as_holding_its_default")
{
    const motion_ops motion{.task_space_pose = &zeroed_pose};
    const std::array<capability_view, 3> reported{view_of(motion), view_of(unbound_trajectory), view_of(unbound_modeling)};
    const capability_view resolution = view_of(motion);

    REQUIRE(count_defaults(reported) == slot_count(reported) - 1u);
    REQUIRE_FALSE(holds_default(resolution, static_cast<std::size_t>(motion_slot::task_space_pose)));
    REQUIRE(slot_name(resolution, static_cast<std::size_t>(motion_slot::task_space_pose)) == "motion.task_space_pose");
    REQUIRE(holds_default(resolution, static_cast<std::size_t>(motion_slot::task_space_screw)));
    REQUIRE(holds_default(view_of(unbound_trajectory), static_cast<std::size_t>(task_trajectory_slot::task_space_waypoints)));
}

TEST_CASE("the_coverage_facility_names_every_slot_of_the_remaining_capabilities")
{
    const capability_view mapping   = view_of(unbound_forward_kinematics);
    const capability_view via_point = view_of(unbound_trajectory);

    REQUIRE(slot_name(mapping, static_cast<std::size_t>(forward_kinematics_slot::forward_kinematics)) == "fk.forward_kinematics");
    REQUIRE(slot_name(mapping, static_cast<std::size_t>(forward_kinematics_slot::body_forward_kinematics)) == "fk.body_forward_kinematics");
    REQUIRE(slot_name(via_point, static_cast<std::size_t>(task_trajectory_slot::task_space_waypoints)) == "trajectory.task_space_waypoints");

    REQUIRE(holds_default(mapping, static_cast<std::size_t>(forward_kinematics_slot::forward_kinematics)));
    REQUIRE(holds_default(mapping, static_cast<std::size_t>(forward_kinematics_slot::body_forward_kinematics)));
    REQUIRE(holds_default(via_point, static_cast<std::size_t>(task_trajectory_slot::task_space_waypoints)));
}

TEST_CASE("the_count_enumerator_names_no_slot_and_enters_no_set")
{
    const capability_view described = view_of(unbound_modeling);
    const std::size_t past          = static_cast<std::size_t>(modeling_slot::count);
    modeling_slot_set expected;

    REQUIRE(slot_name(described, past).empty());
    REQUIRE_FALSE(holds_default(described, past));
    REQUIRE(expected.set(modeling_slot::count).empty());
    REQUIRE_FALSE(expected.contains(modeling_slot::count));
}

TEST_CASE("a_default_constructed_slot_set_is_empty_and_contains_nothing")
{
    motion_slot_set resolution;
    modeling_slot_set described;

    REQUIRE(resolution.empty());
    REQUIRE(described.empty());
    REQUIRE_FALSE(resolution.contains(motion_slot::task_space_screw));
    REQUIRE_FALSE(described.contains(modeling_slot::build_chain));
}

TEST_CASE("slot_sets_compose_by_union_intersection_and_complement")
{
    robot_slot_set framing;
    robot_slot_set solving;

    framing.set(robot_slot::tool_pose_from_flange_pose).set(robot_slot::position_from_pose).set(robot_slot::ik_solve_pose);
    solving.set(robot_slot::ik_solve_pose).set(robot_slot::ik_solve_flange_pose);

    robot_slot_set both   = framing | solving;
    robot_slot_set shared = framing & solving;

    REQUIRE(both.contains(robot_slot::tool_pose_from_flange_pose));
    REQUIRE(both.contains(robot_slot::ik_solve_flange_pose));
    REQUIRE(shared.contains(robot_slot::ik_solve_pose));
    REQUIRE_FALSE(shared.contains(robot_slot::tool_pose_from_flange_pose));
    REQUIRE_FALSE(shared.contains(robot_slot::ik_solve_flange_pose));
    REQUIRE((framing & ~framing).empty());
}

TEST_CASE("complementing_moves_between_the_empty_set_and_the_set_of_every_slot")
{
    const std::uint32_t last          = static_cast<std::uint32_t>(robot_slot::count);
    robot_slot_set complement_of_none = ~robot_slot_set();
    robot_slot_set every;
    std::uint32_t held = 0;

    for(std::uint32_t index = 0; index < last; ++index)
    {
        every.set(static_cast<robot_slot>(index));
        held += complement_of_none.contains(static_cast<robot_slot>(index)) ? 1u : 0u;
    }

    REQUIRE(held == last);
    REQUIRE_FALSE(every.empty());
    REQUIRE((~every).empty());
}
