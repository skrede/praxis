#include "praxis/manipulator/slots.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/motion.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <array>
#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

std::size_t slot_count()
{
    return static_cast<std::size_t>(forward_kinematics_slot::count) + static_cast<std::size_t>(differential_kinematics_slot::count) +
            static_cast<std::size_t>(inverse_kinematics_slot::count) + static_cast<std::size_t>(robot_slot::count) + static_cast<std::size_t>(motion_slot::count) +
            static_cast<std::size_t>(modeling_slot::count) + static_cast<std::size_t>(task_trajectory_slot::count);
}

expected<joint_vector, refusal> held(const kinematics &, const transform &, const joint_vector &j0)
{
    return j0;
}

}

TEST_CASE("the_composed_capabilities_leave_no_slot_holding_its_inert_default")
{
    const capabilities composed                   = baseline();
    const std::array<capability_view, 7> reported = capability_views(composed);

    REQUIRE(count_defaults(reported) == 0u);
}

TEST_CASE("a_value_initialized_capabilities_reports_every_slot_holding_its_inert_default")
{
    const capabilities untouched                  = capabilities{};
    const std::array<capability_view, 7> reported = capability_views(untouched);

    REQUIRE(count_defaults(reported) == slot_count());
    REQUIRE(slot_count() == 18u);
}

TEST_CASE("overriding_a_bound_slot_leaves_it_bound")
{
    capabilities composed                         = baseline();
    composed.motion.task_space_pose               = &held;
    const std::array<capability_view, 7> reported = capability_views(composed);

    CHECK(count_defaults(reported) == 0u);
    CHECK_FALSE(holds_default(reported[4], static_cast<std::size_t>(motion_slot::task_space_pose)));
}

TEST_CASE("every_report_from_an_unbound_capability_carries_this_extension_and_a_unique_slot_name")
{
    const capabilities untouched                  = capabilities{};
    const std::array<capability_view, 7> reported = capability_views(untouched);
    const std::vector<defaulted_slot> held_slots  = defaulted_slots(reported);
    std::set<std::string> qualified;

    REQUIRE(held_slots.size() == slot_count());

    for(const defaulted_slot &entry : held_slots)
    {
        CHECK(entry.extension == "manipulator");
        CHECK_FALSE(entry.slot.empty());
        CHECK(qualified.insert(std::string(entry.extension).append(".").append(entry.slot)).second);
    }
    CHECK(qualified.count("manipulator.fk.forward_kinematics") == 1u);
    CHECK(qualified.count("manipulator.robot.ik_solve_flange_pose") == 1u);
    CHECK(qualified.count("manipulator.modeling.build_chain") == 1u);
    CHECK(qualified.count("manipulator.trajectory.task_space_waypoints") == 1u);
}
