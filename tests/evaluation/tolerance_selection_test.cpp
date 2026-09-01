#include "bent_rigid_motion.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/rigid_motion/evaluation.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <string_view>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

// A displacement of this size is what a second correct implementation of a slot leaves behind, so
// each of these must go unreported: reading one of them as a difference would make every honest
// implementation disagree with the reference. They are fixed rather than derived from the bounds
// above, so a bound moved down onto one of them fails here.
constexpr std::array<double, static_cast<std::size_t>(fixture::bent::count)> beneath_the_bound{1.0e-14, 1.0e-14, 1.0e-14, 1.0e-13, 1.0e-13, 1.0e-14, 1.0e-13};

constexpr std::array<double, static_cast<std::size_t>(fixture::bent::count)> above_the_bound{
        10.0 * element_wise_tolerance,    10.0 * geodesic_tolerance_radians,         10.0 * pose_tolerance_radians,           10.0 * pose_tolerance_metres,
        10.0 * axis_up_to_sign_tolerance, 10.0 * log_up_to_branch_tolerance_radians, 10.0 * log_up_to_branch_tolerance_metres};

// `evaluate` draws from the bulk of each argument role's range; this is the same walk over the same
// tables through the neighbourhood of the singular values instead.
evaluation_report evaluated_near_the_singular(std::span<const evaluation_view> compared, std::uint64_t seed, std::size_t cases)
{
    evaluation_report reported{{}, seed, cases};

    for(const evaluation_view &view : compared)
        for(const slot_evaluation &slot : view.slots())
        {
            residual worst{slot.kind, 0.0, 0.0};
            agreement verdict = agreement::agreed;

            for(std::size_t index = 0; index < cases; ++index)
            {
                case_source drawn      = case_source::at_case(seed, slot.name, spread::near_singular, index);
                const case_result seen = slot.compare(view.first(), view.second(), drawn, slot.allowed);

                if(seen.verdict == agreement::differed)
                    verdict = agreement::differed;
                if(seen.difference.magnitude > worst.magnitude)
                    worst.magnitude = seen.difference.magnitude;
                if(seen.difference.linear_error_metres > worst.linear_error_metres)
                    worst.linear_error_metres = seen.difference.linear_error_metres;
            }

            reported.slots.push_back(slot_report{view.extension(), slot.name, verdict, worst, cases, 0, outcome_counts{}});
        }

    return reported;
}

bool every_worst_is_a_decade_under_its_bound(const evaluation_report &reported)
{
    for(const slot_report &slot : reported.slots)
    {
        const tolerance_pair allowed = tolerance_of(slot.worst.kind);

        if(slot.worst.magnitude * 10.0 > allowed.magnitude || slot.worst.linear_error_metres * 10.0 > allowed.linear_metres)
            return false;
    }

    return true;
}

std::vector<std::string_view> bent_slots(const evaluation_report &reported)
{
    std::vector<std::string_view> named;

    for(const slot_report &slot : reported.slots)
        if(fixture::is_bent(slot.slot))
            named.push_back(slot.slot);

    return named;
}

void bend_every_slot_by(const std::array<double, static_cast<std::size_t>(fixture::bent::count)> &by)
{
    std::copy(by.begin(), by.end(), fixture::bend_by.begin());
}

}

TEST_CASE("the_reference_against_itself_stays_a_decade_under_every_bound_in_both_spreads")
{
    bend_every_slot_by({});

    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, reference);
    const evaluation_report bulk                  = evaluate(compared);
    const evaluation_report singular              = evaluated_near_the_singular(compared, default_seed, default_cases_per_slot);

    REQUIRE(bulk.slots.size() == 29u);
    REQUIRE(bulk.cases_per_slot == default_cases_per_slot);
    REQUIRE(bulk.seed == default_seed);
    REQUIRE(every_worst_is_a_decade_under_its_bound(bulk));
    REQUIRE(every_worst_is_a_decade_under_its_bound(singular));
}

TEST_CASE("a_displacement_beneath_a_bound_is_not_reported_and_one_a_decade_above_it_is")
{
    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const rigid_motion::capabilities bent         = fixture::bent_everywhere();
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, bent);

    bend_every_slot_by(beneath_the_bound);

    const evaluation_report quiet = evaluate(compared);

    bend_every_slot_by(above_the_bound);

    const evaluation_report loud = evaluate(compared);

    bend_every_slot_by({});

    REQUIRE(every_slot_agreed(quiet));
    REQUIRE_FALSE(every_slot_agreed(loud));
    REQUIRE(disagreeing_slots(loud) == bent_slots(loud));
    REQUIRE(disagreeing_slots(loud).size() == static_cast<std::size_t>(fixture::bent::count));
}

TEST_CASE("every_slot_agreed_holds_for_the_reference_against_itself_over_all_twenty_nine_slots")
{
    bend_every_slot_by({});

    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, reference);
    const evaluation_report reported              = evaluate(compared);

    REQUIRE(every_slot_agreed(reported));
    REQUIRE(disagreeing_slots(reported).empty());
}

TEST_CASE("two_runs_at_the_shipped_seed_report_the_same_verdict_and_the_same_residual_slot_for_slot")
{
    bend_every_slot_by({});

    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, reference);
    const evaluation_report reported              = evaluate(compared);
    const evaluation_report again                 = evaluate(compared);

    REQUIRE(reported.slots.size() == again.slots.size());
    for(std::size_t at = 0; at < reported.slots.size(); ++at)
    {
        REQUIRE(reported.slots[at].slot == again.slots[at].slot);
        REQUIRE(reported.slots[at].verdict == again.slots[at].verdict);
        REQUIRE(reported.slots[at].worst.magnitude == again.slots[at].worst.magnitude);
        REQUIRE(reported.slots[at].worst.linear_error_metres == again.slots[at].worst.linear_error_metres);
        REQUIRE(reported.slots[at].worst_case_index == again.slots[at].worst_case_index);
    }
}
