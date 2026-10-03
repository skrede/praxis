#include "book_via_points.h"
#include "evaluation_cases.h"

#include "praxis/trajectory/evaluation.h"
#include "praxis/trajectory/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::evaluation;

namespace {

constexpr std::size_t fewest_held_runs = 50u;

// Only the joint-space trajectory view is evaluated: it is the last the aggregate lists.
slot_report judged(via_point_slot slot)
{
    trajectory::capabilities literal         = trajectory::baseline();
    literal.trajectory.joint_space_waypoints = slot;
    const trajectory::capabilities reference = trajectory::baseline();
    const auto views                         = trajectory::evaluation_views(literal, reference);

    return evaluate(std::span(views).last(1), default_seed, default_cases_per_slot).slots.at(0);
}

void require_agreed(const slot_report &row)
{
    REQUIRE(row.slot == "trajectory.joint_space_waypoints");
    REQUIRE(row.verdict == agreement::agreed);
    REQUIRE(row.outcomes.agreed == default_cases_per_slot);
}

// Some coordinate of a row after the first equals the row before it exactly, while the row itself
// moves.
bool holds_a_joint_still(const std::vector<trajectory::configuration> &rows)
{
    for(std::size_t k = 1; k < rows.size(); ++k)
        if(rows[k] != rows[k - 1] && (rows[k].array() == rows[k - 1].array()).any())
            return true;

    return false;
}

}

TEST_CASE("every_literal_via_point_run_held_at_its_ends_is_judged_agreeing_by_the_joint_space_waypoint_row")
{
    const auto slots = book_via_point_slots<via_forms>();
    for(std::size_t form = 0; form < slots.size(); ++form)
    {
        if(via_forms.at(form).ends != via_ends::held)
            continue;

        const slot_report row = judged(slots.at(form));

        INFO("form " << form);
        require_agreed(row);
    }
}

TEST_CASE("every_literal_via_point_run_at_rest_beyond_its_ends_is_judged_agreeing_by_the_joint_space_waypoint_row")
{
    const auto slots = book_via_point_slots<via_forms>();
    for(std::size_t form = 0; form < slots.size(); ++form)
    {
        if(via_forms.at(form).ends != via_ends::at_rest)
            continue;

        const slot_report row = judged(slots.at(form));

        INFO("form " << form);
        require_agreed(row);
    }
}

TEST_CASE("a_via_point_run_whose_sign_test_gives_a_zero_slope_the_sign_of_a_neighbour_is_judged_differing_by_the_joint_space_waypoint_row")
{
    const auto slots = book_via_point_slots<via_sign_controls>();
    for(std::size_t control = 0; control < slots.size(); ++control)
    {
        const slot_report row = judged(slots.at(control));

        INFO("control " << control);
        REQUIRE(row.verdict != agreement::agreed);
        REQUIRE(row.outcomes.differed > 0u);
    }
}

TEST_CASE("the_joint_space_waypoint_row_holds_one_joint_still_across_one_segment_in_at_least_fifty_cases_at_the_default_draw")
{
    std::size_t held = 0;
    for(std::size_t index = 0; index < default_cases_per_slot; ++index)
    {
        case_source drawn                         = case_source::at_case(default_seed, "trajectory.joint_space_waypoints", spread::bulk, index);
        const trajectory::joint_waypoint_case run = trajectory::drawn_joint_waypoint_case(drawn);
        held += holds_a_joint_still(run.waypoints) ? 1u : 0u;
    }

    REQUIRE(held >= fewest_held_runs);
}
