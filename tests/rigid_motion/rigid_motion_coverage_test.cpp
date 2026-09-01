#include "praxis/rigid_motion.h"

#include "praxis/rigid_motion/slots.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <array>
#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

matrix3 constant_skew(const Eigen::Vector3d &)
{
    return matrix3::Constant(4.0);
}

std::size_t frame_slots()
{
    return static_cast<std::size_t>(frame_slot::count);
}

std::size_t screw_slots()
{
    return static_cast<std::size_t>(screw_slot::count);
}

std::array<capability_view, 2> unbound_views(const frame_ops &frame, const screw_ops &screw)
{
    return {view_of(frame), view_of(screw)};
}

}

TEST_CASE("the_descriptor_tables_cover_every_slot_of_both_capabilities_under_a_unique_non_empty_name")
{
    const frame_ops frame{};
    const screw_ops screw{};
    const std::array<capability_view, 2> views = unbound_views(frame, screw);
    std::set<std::string> names;

    REQUIRE(views.front().slots().size() == frame_slots());
    REQUIRE(views.back().slots().size() == screw_slots());

    for(const capability_view &view : views)
    {
        for(const slot_descriptor &descriptor : view.slots())
        {
            REQUIRE_FALSE(descriptor.name.empty());
            REQUIRE(names.insert(std::string(descriptor.name)).second);
        }
    }
    REQUIRE(names.size() == frame_slots() + screw_slots());
}

TEST_CASE("a_value_initialized_capability_reports_every_slot_as_holding_its_default")
{
    const frame_ops frame{};
    const screw_ops screw{};
    const std::array<capability_view, 2> views = unbound_views(frame, screw);

    REQUIRE(count_defaults(views) == frame_slots() + screw_slots());

    for(std::size_t index = 0; index < frame_slots(); ++index)
    {
        REQUIRE(holds_default(views.front(), index));
    }
    for(std::size_t index = 0; index < screw_slots(); ++index)
    {
        REQUIRE(holds_default(views.back(), index));
    }
}

TEST_CASE("overriding_one_slot_leaves_every_other_slot_reported_as_holding_its_default")
{
    const frame_ops frame{};
    const screw_ops screw{.skew_symmetric = &constant_skew};
    const std::array<capability_view, 2> views = unbound_views(frame, screw);

    REQUIRE(count_defaults(views) == frame_slots() + screw_slots() - 1u);
    REQUIRE_FALSE(holds_default(views.back(), static_cast<std::size_t>(screw_slot::skew_symmetric)));
    REQUIRE(holds_default(views.front(), static_cast<std::size_t>(frame_slot::rotate_z)));
    REQUIRE(slot_name(views.back(), static_cast<std::size_t>(screw_slot::skew_symmetric)) == "screw.skew_symmetric");
    REQUIRE(slot_name(views.front(), static_cast<std::size_t>(frame_slot::rotate_z)) == "frame.rotate_z");
}

TEST_CASE("the_report_labels_the_slots_of_both_capabilities_with_this_extension")
{
    const frame_ops frame{};
    const screw_ops screw{};
    const std::array<capability_view, 2> views = unbound_views(frame, screw);
    const std::vector<defaulted_slot> report   = defaulted_slots(views);
    std::set<std::string> qualified;

    REQUIRE(report.size() == frame_slots() + screw_slots());
    REQUIRE(count_defaults(views) == report.size());

    for(const defaulted_slot &entry : report)
    {
        REQUIRE(entry.extension == "rigid_motion");
        REQUIRE(qualified.insert(std::string(entry.extension).append(".").append(entry.slot)).second);
    }
    REQUIRE(qualified.count("rigid_motion.frame.rotate_z") == 1u);
    REQUIRE(qualified.count("rigid_motion.screw.adjoint_map") == 1u);
}
