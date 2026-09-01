#include "praxis/trajectory/slots.h"
#include "praxis/trajectory/capabilities.h"
#include "praxis/trajectory/baseline/path.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <array>
#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::trajectory;

namespace {

std::size_t slot_count()
{
    return static_cast<std::size_t>(time_scaling_slot::count) + static_cast<std::size_t>(path_slot::count) + static_cast<std::size_t>(pose_trajectory_slot::count) +
            static_cast<std::size_t>(trajectory_slot::count);
}

expected<transform, refusal> held(const transform &start, const transform &, double)
{
    return start;
}

}

TEST_CASE("the_composed_capabilities_leave_no_slot_holding_its_inert_default")
{
    const capabilities composed                   = baseline();
    const std::array<capability_view, 4> reported = capability_views(composed);

    REQUIRE(count_defaults(reported) == 0u);
}

TEST_CASE("a_value_initialized_capabilities_reports_every_slot_holding_its_inert_default")
{
    const capabilities untouched                  = capabilities{};
    const std::array<capability_view, 4> reported = capability_views(untouched);

    REQUIRE(count_defaults(reported) == slot_count());
    REQUIRE(slot_count() == 9u);
}

TEST_CASE("overriding_a_bound_slot_leaves_it_bound")
{
    capabilities composed                         = baseline();
    composed.path.screw                           = &held;
    const std::array<capability_view, 4> reported = capability_views(composed);

    CHECK(count_defaults(reported) == 0u);
    CHECK_FALSE(holds_default(reported[1], static_cast<std::size_t>(path_slot::screw)));
}

TEST_CASE("every_report_from_an_unbound_capability_carries_this_extension_and_a_unique_slot_name")
{
    const capabilities untouched                  = capabilities{};
    const std::array<capability_view, 4> reported = capability_views(untouched);
    const std::vector<defaulted_slot> held_slots  = defaulted_slots(reported);
    std::set<std::string> qualified;

    REQUIRE(held_slots.size() == slot_count());

    for(const defaulted_slot &entry : held_slots)
    {
        CHECK(entry.extension == "trajectory");
        CHECK_FALSE(entry.slot.empty());
        CHECK(qualified.insert(std::string(entry.extension).append(".").append(entry.slot)).second);
    }
    CHECK(qualified.count("trajectory.time_scaling.quintic") == 1u);
    CHECK(qualified.count("trajectory.pose_trajectory.decoupled_pose_waypoints") == 1u);
    CHECK(qualified.count("trajectory.trajectory.joint_space_waypoints") == 1u);
}
