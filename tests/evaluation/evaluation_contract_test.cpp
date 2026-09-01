#include "praxis/evaluation/report.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/rigid_motion/baseline/frame.h"

#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/evaluation.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <array>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <string_view>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr std::uint64_t recorded_seed = 0x5EEDu;

rotation bent_rotate_x(double radians)
{
    return rigid_motion::rotate_x(radians + 1.0);
}

rigid_motion::capabilities bent()
{
    rigid_motion::capabilities spatial = rigid_motion::baseline();
    spatial.frame.rotate_x             = &bent_rotate_x;

    return spatial;
}

std::size_t every_slot()
{
    return static_cast<std::size_t>(rigid_motion::frame_slot::count) + static_cast<std::size_t>(rigid_motion::screw_slot::count);
}

std::vector<std::string_view> reported_in_order(const evaluation_report &reported)
{
    std::vector<std::string_view> named;

    for(const slot_report &slot : reported.slots)
        named.push_back(slot.slot);

    return named;
}

bool names(const std::vector<std::string_view> &reported, std::string_view slot)
{
    return std::find(reported.begin(), reported.end(), slot) != reported.end();
}

const slot_report &reported_for(const evaluation_report &reported, std::string_view slot)
{
    const auto found = std::find_if(reported.slots.begin(), reported.slots.end(), [slot](const slot_report &row) { return row.slot == slot; });

    REQUIRE(found != reported.slots.end());

    return *found;
}

}

TEST_CASE("a_reference_evaluated_against_itself_agrees_on_every_slot_and_answers_the_same_on_a_repeated_run")
{
    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, reference);
    const evaluation_report reported              = evaluate(compared, recorded_seed, 16);
    const evaluation_report again                 = evaluate(compared, recorded_seed, 16);
    const slot_report &turned                     = reported_for(reported, "frame.rotate_x");

    REQUIRE(reported.slots.size() == every_slot());
    REQUIRE(turned.extension == "rigid_motion");
    REQUIRE(turned.verdict == agreement::agreed);
    REQUIRE(turned.worst.magnitude == 0.0);
    REQUIRE(turned.cases == 16u);
    REQUIRE(reported.seed == recorded_seed);
    REQUIRE(every_slot_agreed(reported));
    REQUIRE(disagreeing_slots(reported).empty());
    REQUIRE(reported_in_order(again) == reported_in_order(reported));
}

TEST_CASE("a_bent_slot_is_named_and_carries_a_residual_the_size_of_the_bend")
{
    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const rigid_motion::capabilities subject      = bent();
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, subject);
    const evaluation_report reported              = evaluate(compared, recorded_seed, 16);

    const slot_report &turned = reported_for(reported, "frame.rotate_x");

    REQUIRE_FALSE(every_slot_agreed(reported));
    REQUIRE(disagreeing_slots(reported) == std::vector<std::string_view>{"frame.rotate_x"});
    REQUIRE(turned.verdict == agreement::differed);
    REQUIRE(turned.worst.kind == residual_kind::geodesic);
    REQUIRE(std::fabs(turned.worst.magnitude - 1.0) < 1.0e-9);
    REQUIRE(turned.worst_case_index < 16u);
}

TEST_CASE("no_slot_the_seam_declares_is_left_for_the_report_to_pass_over_in_silence")
{
    const rigid_motion::capabilities reference     = rigid_motion::baseline();
    const std::array<capability_view, 2> described = rigid_motion::capability_views(reference);
    const std::array<evaluation_view, 2> compared  = rigid_motion::evaluation_views(reference, reference);

    REQUIRE(unevaluated_slots(described, compared).empty());
    REQUIRE(unnamed_evaluations(described, compared).empty());
}

TEST_CASE("a_run_of_no_cases_reports_every_slot_as_unexercised_and_never_as_a_pass")
{
    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, reference);
    const evaluation_report reported              = evaluate(compared, recorded_seed, 0);

    REQUIRE(reported.cases_per_slot == 0u);
    for(const slot_report &slot : reported.slots)
        REQUIRE(slot.verdict == agreement::not_exercised);

    REQUIRE_FALSE(every_slot_agreed(reported));
    REQUIRE(names(disagreeing_slots(reported), "frame.rotate_x"));
}
