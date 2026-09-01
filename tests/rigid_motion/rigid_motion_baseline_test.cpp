#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <array>
#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

std::size_t frame_slots()
{
    return static_cast<std::size_t>(frame_slot::count);
}

std::size_t screw_slots()
{
    return static_cast<std::size_t>(screw_slot::count);
}

matrix3 constant_skew(const Eigen::Vector3d &)
{
    return matrix3::Constant(4.0);
}

}

TEST_CASE("the_composed_capabilities_leave_no_slot_holding_its_inert_default")
{
    const capabilities composed                   = baseline();
    const std::array<capability_view, 2> reported = capability_views(composed);

    REQUIRE(count_defaults(reported) == 0u);
}

TEST_CASE("a_value_initialized_capabilities_reports_every_slot_of_both_capabilities_holding_its_inert_default")
{
    const capabilities untouched                  = capabilities{};
    const std::array<capability_view, 2> reported = capability_views(untouched);

    REQUIRE(reported.front().slots().size() == frame_slots());
    REQUIRE(reported.back().slots().size() == screw_slots());
    REQUIRE(count_defaults(reported) == frame_slots() + screw_slots());
    REQUIRE(frame_slots() > 0u);
    REQUIRE(screw_slots() > 0u);
}

TEST_CASE("overriding_a_bound_slot_leaves_it_bound")
{
    capabilities composed                         = baseline();
    composed.screw.skew_symmetric                 = &constant_skew;
    const std::array<capability_view, 2> reported = capability_views(composed);

    CHECK(count_defaults(reported) == 0u);
    CHECK_FALSE(holds_default(reported.back(), static_cast<std::size_t>(screw_slot::skew_symmetric)));
}

TEST_CASE("every_report_from_an_unbound_capability_carries_this_extension_and_a_unique_slot_name")
{
    const capabilities untouched                  = capabilities{};
    const std::array<capability_view, 2> reported = capability_views(untouched);
    const std::vector<defaulted_slot> held        = defaulted_slots(reported);
    std::set<std::string> qualified;

    REQUIRE(held.size() == frame_slots() + screw_slots());

    for(const defaulted_slot &entry : held)
    {
        CHECK(entry.extension == "rigid_motion");
        CHECK_FALSE(entry.slot.empty());
        CHECK(qualified.insert(std::string(entry.extension).append(".").append(entry.slot)).second);
    }
    CHECK(qualified.count("rigid_motion.frame.rotate_z") == 1u);
    CHECK(qualified.count("rigid_motion.screw.adjoint_map") == 1u);
}
