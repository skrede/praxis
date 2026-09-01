#include "praxis/trajectory/path.h"
#include "praxis/trajectory/types.h"
#include "praxis/trajectory/evaluation.h"
#include "praxis/trajectory/capabilities.h"
#include "praxis/trajectory/time_scaling.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/rigid_motion/types.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr std::uint64_t recorded_seed = 0xC0FFEEu;
constexpr std::size_t cases_per_row   = 24u;
constexpr std::size_t compared_rows   = 9u;

// The rows answering a value rather than a prepared generator are the first six the report lists.
// The three after them hand back an object and decline through it instead.
constexpr std::size_t value_rows = 6u;

// Wide enough that the bounded profile reaches both sides of its own feasibility at the shipped
// draw, which is what the last case below reads.
constexpr std::size_t wider_run = 200u;

expected<trajectory::scaling_sample, refusal> degenerate_scaling(double, double)
{
    return unexpected(refusal::degenerate);
}

expected<trajectory::scaling_sample, refusal> degenerate_trapezoidal(double, double, double, double)
{
    return unexpected(refusal::degenerate);
}

expected<trajectory::configuration, refusal> degenerate_joint_path(const trajectory::configuration &, const trajectory::configuration &, double)
{
    return unexpected(refusal::degenerate);
}

expected<transform, refusal> degenerate_pose_path(const transform &, const transform &, double)
{
    return unexpected(refusal::degenerate);
}

// Every slot declines, and every one of them declines for a reason the inert bindings never give.
trajectory::capabilities declining_for_another_reason()
{
    trajectory::capabilities shapes{};

    shapes.time_scaling.cubic       = &degenerate_scaling;
    shapes.time_scaling.quintic     = &degenerate_scaling;
    shapes.time_scaling.trapezoidal = &degenerate_trapezoidal;
    shapes.path.joint_straight_line = &degenerate_joint_path;
    shapes.path.screw               = &degenerate_pose_path;
    shapes.path.decoupled           = &degenerate_pose_path;

    return shapes;
}

evaluation_report reported_against(const trajectory::capabilities &reference, const trajectory::capabilities &other)
{
    return evaluate(trajectory::evaluation_views(reference, other), recorded_seed, cases_per_row);
}

bool carries_no_number(const residual &worst)
{
    return worst.magnitude == 0.0 && worst.linear_error_metres == 0.0;
}

}

TEST_CASE("two_bindings_declining_alike_agree_about_the_input_they_both_declined")
{
    const trajectory::capabilities inert{};
    const evaluation_report reported = reported_against(inert, inert);

    REQUIRE(reported.slots.size() == compared_rows);
    for(const slot_report &row : reported.slots)
    {
        INFO(row.slot);
        REQUIRE(row.verdict == agreement::both_refused);
        REQUIRE(row.outcomes.both_refused == cases_per_row);
        REQUIRE(carries_no_number(row.worst));
    }
}

TEST_CASE("two_bindings_declining_for_different_reasons_reach_an_outcome_of_their_own")
{
    const trajectory::capabilities inert{};
    const evaluation_report reported = reported_against(inert, declining_for_another_reason());

    REQUIRE(reported.slots.size() == compared_rows);
    for(std::size_t at = 0; at < value_rows; ++at)
    {
        const slot_report &row = reported.slots.at(at);

        INFO(row.slot);
        REQUIRE(row.verdict == agreement::refused_differently);
        REQUIRE(row.outcomes.refused_differently == cases_per_row);
        REQUIRE(carries_no_number(row.worst));
    }
}

TEST_CASE("a_case_exactly_one_side_declines_is_counted_and_folds_nothing")
{
    const trajectory::capabilities reference = trajectory::baseline();
    const trajectory::capabilities inert{};
    const evaluation_report reported = reported_against(reference, inert);

    REQUIRE(reported.slots.size() == compared_rows);
    for(std::size_t at = 0; at < value_rows; ++at)
    {
        const slot_report &row = reported.slots.at(at);

        INFO(row.slot);
        REQUIRE(row.outcomes.agreed == 0u);
        REQUIRE(row.outcomes.differed == 0u);
        REQUIRE(row.outcomes.unusable == 0u);
        REQUIRE(row.outcomes.one_refused > 0u);
        REQUIRE(row.outcomes.one_refused + row.outcomes.refused_differently == cases_per_row);
        REQUIRE(carries_no_number(row.worst));
    }
}

TEST_CASE("the_row_over_the_bounded_profile_reaches_a_duration_its_bounds_can_stretch_to_and_one_they_cannot")
{
    const trajectory::capabilities reference = trajectory::baseline();
    const evaluation_report reported         = evaluate(trajectory::evaluation_views(reference, reference), recorded_seed, wider_run);
    const slot_report &bounded               = reported.slots.at(2);

    REQUIRE(bounded.slot == "time_scaling.trapezoidal");
    REQUIRE(bounded.outcomes.agreed > 0u);
    REQUIRE(bounded.outcomes.both_refused > 0u);
    REQUIRE(bounded.outcomes.agreed + bounded.outcomes.both_refused == wider_run);
}

TEST_CASE("an_unbound_generator_slot_against_a_bound_one_is_a_difference_in_what_the_two_span")
{
    const trajectory::capabilities reference = trajectory::baseline();
    const trajectory::capabilities inert{};
    const evaluation_report reported = reported_against(reference, inert);

    for(std::size_t row = value_rows; row < reported.slots.size(); ++row)
    {
        INFO(reported.slots.at(row).slot);
        REQUIRE(reported.slots.at(row).verdict == agreement::differed);
        REQUIRE(reported.slots.at(row).outcomes.differed == cases_per_row);
        REQUIRE(reported.slots.at(row).outcomes.agreed == 0u);
        REQUIRE(std::isinf(reported.slots.at(row).worst.magnitude));
    }
}
