#include "praxis/evaluation.h"

#include "praxis/rigid_motion/evaluation.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/baseline/frame.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <string_view>

// A project outside this repository composing its own comparison from the shipped facility: one
// composition stands for the reference and one for the subject, and the facility answers per slot.
namespace outside {

constexpr std::uint64_t recorded_seed = 0x0C0FFEEull;
constexpr std::size_t cases_per_slot  = 32;

praxis::rotation turned_further(double radians)
{
    return praxis::rigid_motion::rotate_x(radians + 0.25);
}

praxis::rigid_motion::capabilities reference()
{
    return praxis::rigid_motion::baseline();
}

praxis::rigid_motion::capabilities subject_that_matches()
{
    return praxis::rigid_motion::baseline();
}

praxis::rigid_motion::capabilities subject_that_differs()
{
    praxis::rigid_motion::capabilities composed = praxis::rigid_motion::baseline();
    composed.frame.rotate_x                     = &turned_further;

    return composed;
}

bool names(const std::vector<std::string_view> &reported, std::string_view slot)
{
    return std::find(reported.begin(), reported.end(), slot) != reported.end();
}

std::size_t declared(const std::array<praxis::capability_view, 2> &described)
{
    std::size_t counted = 0;

    for(const praxis::capability_view &view : described)
        counted += view.slots().size();

    return counted;
}

double worst_on(const praxis::evaluation::evaluation_report &reported, std::string_view slot)
{
    for(const praxis::evaluation::slot_report &row : reported.slots)
        if(row.slot == slot)
            return row.worst.magnitude;

    return 0.0;
}

}

TEST_CASE("a_subject_answering_as_the_reference_does_agrees_on_every_slot_the_facility_compares")
{
    const praxis::rigid_motion::capabilities held                     = outside::reference();
    const praxis::rigid_motion::capabilities offered                  = outside::subject_that_matches();
    const std::array<praxis::evaluation::evaluation_view, 2> compared = praxis::rigid_motion::evaluation_views(held, offered);
    const praxis::evaluation::evaluation_report reported              = praxis::evaluation::evaluate(compared, outside::recorded_seed, outside::cases_per_slot);

    REQUIRE(praxis::evaluation::every_slot_agreed(reported));
    REQUIRE(praxis::evaluation::disagreeing_slots(reported).empty());
}

TEST_CASE("a_subject_answering_differently_is_named_by_the_slot_it_differs_on")
{
    const praxis::rigid_motion::capabilities held                     = outside::reference();
    const praxis::rigid_motion::capabilities offered                  = outside::subject_that_differs();
    const std::array<praxis::evaluation::evaluation_view, 2> compared = praxis::rigid_motion::evaluation_views(held, offered);
    const praxis::evaluation::evaluation_report reported              = praxis::evaluation::evaluate(compared, outside::recorded_seed, outside::cases_per_slot);

    REQUIRE_FALSE(praxis::evaluation::every_slot_agreed(reported));
    REQUIRE(outside::names(praxis::evaluation::disagreeing_slots(reported), "frame.rotate_x"));
    REQUIRE(outside::worst_on(reported, "frame.rotate_x") > praxis::default_tolerance);
}

TEST_CASE("the_shipped_tables_leave_no_slot_uncompared_and_name_none_the_seam_does_not_declare")
{
    const praxis::rigid_motion::capabilities held                     = outside::reference();
    const std::array<praxis::capability_view, 2> described            = praxis::rigid_motion::capability_views(held);
    const std::array<praxis::evaluation::evaluation_view, 2> compared = praxis::rigid_motion::evaluation_views(held, held);
    const std::vector<std::string_view> uncovered                     = praxis::evaluation::unevaluated_slots(described, compared);

    REQUIRE(uncovered.empty());
    REQUIRE(praxis::evaluation::unnamed_evaluations(described, compared).empty());
}

TEST_CASE("the_composition_carries_one_verdict_per_slot_over_every_slot_the_shipped_seam_declares")
{
    const praxis::rigid_motion::capabilities held                     = outside::reference();
    const praxis::rigid_motion::capabilities offered                  = outside::subject_that_differs();
    const std::array<praxis::capability_view, 2> described            = praxis::rigid_motion::capability_views(held);
    const std::array<praxis::evaluation::evaluation_view, 2> compared = praxis::rigid_motion::evaluation_views(held, offered);
    const praxis::evaluation::evaluation_report reported              = praxis::evaluation::evaluate(compared, outside::recorded_seed, outside::cases_per_slot);

    REQUIRE(reported.slots.size() == outside::declared(described));
    REQUIRE(reported.slots.size() == 29u);
    for(const praxis::evaluation::slot_report &row : reported.slots)
    {
        REQUIRE_FALSE(row.slot.empty());
        REQUIRE(row.extension == "rigid_motion");
        REQUIRE(row.cases == outside::cases_per_slot);
    }
    REQUIRE(praxis::evaluation::disagreeing_slots(reported) == std::vector<std::string_view>{"frame.rotate_x"});
}
