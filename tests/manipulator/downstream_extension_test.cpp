#include "praxis/manipulator.h"

#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/frame.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <array>
#include <string>
#include <cstddef>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::rigid_motion;

namespace {

double unit_pitch()
{
    return 1.0;
}

// A capability an extension outside this repository could declare, spelling one of its slots exactly
// the way a shipped extension spells one of its own.
struct mirror_ops
{
    double (*task_space_screw)() = &unit_pitch;
};

constexpr std::array mirror_descriptors{
        slot_descriptor{"motion.task_space_screw", [](const void *value) -> bool { return static_cast<const mirror_ops *>(value)->task_space_screw == &unit_pitch; }},
};

constexpr capability_descriptors<mirror_ops> described_mirrors{"downstream", mirror_descriptors};

capability_view view_of(const mirror_ops &ops)
{
    return capability_view::of(ops, described_mirrors);
}

const motion_ops unbound_motion{};
const frame_ops unbound_frames{};
const mirror_ops unbound_mirror{};

}

TEST_CASE("two_extensions_spelling_a_slot_the_same_way_stay_distinct_under_their_own_labels")
{
    const std::array<capability_view, 3> reported{view_of(unbound_motion), view_of(unbound_frames), view_of(unbound_mirror)};
    const std::size_t shipped = static_cast<std::size_t>(motion_slot::count) + static_cast<std::size_t>(frame_slot::count);
    std::set<std::string> pairs;
    std::size_t sharing = 0;

    for(const defaulted_slot &entry : defaulted_slots(reported))
    {
        sharing += entry.slot == "motion.task_space_screw" ? 1u : 0u;
        REQUIRE(pairs.insert(std::string(entry.extension).append(".").append(entry.slot)).second);
    }
    REQUIRE(sharing == 2u);
    REQUIRE(pairs.size() == shipped + 1u);
    REQUIRE(pairs.count("manipulator.motion.task_space_screw") == 1u);
    REQUIRE(pairs.count("downstream.motion.task_space_screw") == 1u);
}
