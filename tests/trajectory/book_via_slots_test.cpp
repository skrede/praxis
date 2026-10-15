#include "book_via_slots.h"
#include "book_via_schedules.h"

#include "praxis/trajectory/evaluation.h"
#include "praxis/trajectory/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <cstddef>
#include <cstdint>
#include <string_view>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::evaluation;

namespace {

constexpr std::size_t seeds_walked  = 40u;
constexpr std::size_t via_point_row = 0u;
constexpr std::size_t schedule_row  = 1u;
constexpr std::size_t printed_form  = 0u;

// Only the joint-space trajectory view is evaluated: it is the last the aggregate lists.
slot_report judged(const trajectory::capabilities &literal, std::size_t row, std::uint64_t seed)
{
    const trajectory::capabilities reference = trajectory::baseline();
    const auto views                         = trajectory::evaluation_views(literal, reference);

    return evaluate(std::span(views).last(1), seed, default_cases_per_slot).slots.at(row);
}

slot_report judged_via_slot(via_slot slot, std::uint64_t seed)
{
    trajectory::capabilities literal          = trajectory::baseline();
    literal.trajectory.joint_space_via_points = slot;

    return judged(literal, via_point_row, seed);
}

slot_report judged_schedule(schedule_slot slot, std::uint64_t seed)
{
    trajectory::capabilities literal      = trajectory::baseline();
    literal.trajectory.via_point_schedule = slot;

    return judged(literal, schedule_row, seed);
}

void require_agreed(const slot_report &row, std::string_view slot)
{
    REQUIRE(row.slot == slot);
    REQUIRE(row.verdict == agreement::agreed);
    REQUIRE(row.outcomes.agreed == default_cases_per_slot);
}

}

TEST_CASE("every_literal_via_point_slot_is_judged_agreeing_by_the_via_point_row")
{
    const auto slots = literal_via_slots();
    for(std::size_t form = 0; form < slots.size(); ++form)
    {
        INFO("form " << form);
        require_agreed(judged_via_slot(slots.at(form), default_seed), "trajectory.joint_space_via_points");
    }
}

TEST_CASE("the_printed_via_point_slot_is_judged_agreeing_by_the_via_point_row_at_each_of_forty_seeds")
{
    const via_slot printed = literal_via_slots().at(printed_form);
    for(std::size_t offset = 1; offset <= seeds_walked; ++offset)
    {
        INFO("seed offset " << offset);
        require_agreed(judged_via_slot(printed, default_seed + offset), "trajectory.joint_space_via_points");
    }
}

TEST_CASE("every_literal_schedule_is_judged_agreeing_by_the_schedule_row")
{
    const auto slots = literal_schedules<schedule_forms>();
    for(std::size_t form = 0; form < slots.size(); ++form)
    {
        INFO("form " << form);
        require_agreed(judged_schedule(slots.at(form), default_seed), "trajectory.via_point_schedule");
    }
}

TEST_CASE("the_printed_schedule_in_both_short_run_spellings_is_judged_agreeing_by_the_schedule_row_at_each_of_forty_seeds")
{
    const auto slots = literal_schedules<schedule_forms>();
    for(std::size_t offset = 1; offset <= seeds_walked; ++offset)
        for(std::size_t form : {0u, 1u})
        {
            INFO("seed offset " << offset << ", form " << form);
            require_agreed(judged_schedule(slots.at(form), default_seed + offset), "trajectory.via_point_schedule");
        }
}

TEST_CASE("a_schedule_whose_sign_test_gives_a_zero_slope_the_sign_of_a_neighbour_is_judged_differing_by_the_schedule_row")
{
    const auto slots = literal_schedules<schedule_sign_controls>();
    for(std::size_t control = 0; control < slots.size(); ++control)
    {
        const slot_report row = judged_schedule(slots.at(control), default_seed);

        INFO("control " << control);
        REQUIRE(row.verdict != agreement::agreed);
        REQUIRE(row.outcomes.differed > 0u);
    }
}
