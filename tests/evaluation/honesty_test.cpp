#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/rigid_motion/baseline/frame.h"

#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/evaluation.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/extension/refusal.h"
#include "praxis/extension/coverage.h"

#include "praxis/compat/expected.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

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
constexpr std::size_t sampled_cases   = 16;
constexpr std::string_view declining  = "screw.adjoint_matrix_from_transform";
constexpr std::size_t turned_slot     = static_cast<std::size_t>(rigid_motion::frame_slot::rotate_x);

std::size_t every_slot()
{
    return static_cast<std::size_t>(rigid_motion::frame_slot::count) + static_cast<std::size_t>(rigid_motion::screw_slot::count);
}

rotation turned_further(double radians)
{
    return rigid_motion::rotate_x(radians + 1.0);
}

rigid_motion::capabilities bound_wrongly()
{
    rigid_motion::capabilities spatial = rigid_motion::baseline();
    spatial.frame.rotate_x             = &turned_further;

    return spatial;
}

expected<adjoint, refusal> declines_as_degenerate(const transform &)
{
    return praxis::unexpected(refusal::degenerate);
}

rigid_motion::capabilities declining_by(expected<adjoint, refusal> (*how)(const transform &tf))
{
    rigid_motion::capabilities spatial          = rigid_motion::baseline();
    spatial.screw.adjoint_matrix_from_transform = how;

    return spatial;
}

evaluation_report judged(const rigid_motion::capabilities &held, const rigid_motion::capabilities &against)
{
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(held, against);

    return evaluate(compared, recorded_seed, sampled_cases);
}

slot_report row_for(const evaluation_report &reported, std::string_view slot)
{
    const auto found = std::find_if(reported.slots.begin(), reported.slots.end(), [slot](const slot_report &row) { return row.slot == slot; });

    REQUIRE(found != reported.slots.end());

    return *found;
}

bool names(const std::vector<std::string_view> &reported, std::string_view slot)
{
    return std::find(reported.begin(), reported.end(), slot) != reported.end();
}

bool every_slot_holds_its_default(const std::array<capability_view, 2> &views)
{
    for(const capability_view &view : views)
        for(std::size_t index = 0; index < view.slots().size(); ++index)
            if(!holds_default(view, index))
                return false;

    return true;
}

bool no_residual_was_produced(const slot_report &row)
{
    return row.worst.magnitude == 0.0 && row.worst.linear_error_metres == 0.0;
}

}

TEST_CASE("a_slot_nobody_bound_is_reported_as_unbound_rather_than_as_a_slot_bound_wrongly")
{
    const rigid_motion::capabilities unbound{};
    const rigid_motion::capabilities wrong                = bound_wrongly();
    const std::array<capability_view, 2> nothing_bound    = rigid_motion::capability_views(unbound);
    const std::array<capability_view, 2> everything_bound = rigid_motion::capability_views(wrong);

    REQUIRE(count_defaults(nothing_bound) == every_slot());
    REQUIRE(defaulted_slots(nothing_bound).size() == every_slot());
    REQUIRE(every_slot_holds_its_default(nothing_bound));
    REQUIRE(count_defaults(everything_bound) == 0u);
    REQUIRE_FALSE(holds_default(everything_bound.front(), turned_slot));
}

TEST_CASE("the_verdict_alone_does_not_separate_an_unbound_slot_from_a_wrongly_bound_one")
{
    const rigid_motion::capabilities reference = rigid_motion::baseline();
    const rigid_motion::capabilities wrong     = bound_wrongly();
    const rigid_motion::capabilities unbound{};

    // Both reports decline to call the slot agreed and neither says which of the two it is, so a
    // consumer that wants to tell them apart reads the seam's default predicate over the subject
    // rather than the residual the comparison produced.
    REQUIRE(names(disagreeing_slots(judged(reference, unbound)), "frame.rotate_x"));
    REQUIRE(names(disagreeing_slots(judged(reference, wrong)), "frame.rotate_x"));
    REQUIRE(holds_default(rigid_motion::capability_views(unbound).front(), turned_slot));
    REQUIRE_FALSE(holds_default(rigid_motion::capability_views(wrong).front(), turned_slot));
}

TEST_CASE("a_run_of_no_cases_reports_every_slot_as_not_exercised_and_never_as_a_pass")
{
    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, reference);
    const evaluation_report reported              = evaluate(compared, recorded_seed, 0);

    REQUIRE(reported.slots.size() == every_slot());
    for(const slot_report &row : reported.slots)
    {
        REQUIRE(row.verdict == agreement::not_exercised);
        REQUIRE(row.cases == 0u);
        REQUIRE(no_residual_was_produced(row));
    }
    REQUIRE_FALSE(every_slot_agreed(reported));
    REQUIRE(disagreeing_slots(reported).size() == every_slot());
}

TEST_CASE("two_bindings_that_declined_are_reported_as_having_declined_never_as_a_number_and_never_as_a_pass")
{
    const rigid_motion::capabilities answering = rigid_motion::baseline();
    const rigid_motion::capabilities alike     = declining_by(&rigid_motion::inert::adjoint_matrix_from_transform);
    const rigid_motion::capabilities apart     = declining_by(&declines_as_degenerate);
    const slot_report both                     = row_for(judged(alike, alike), declining);
    const slot_report differently              = row_for(judged(alike, apart), declining);
    const slot_report one                      = row_for(judged(answering, alike), declining);

    REQUIRE(both.verdict == agreement::both_refused);
    REQUIRE(differently.verdict == agreement::refused_differently);
    REQUIRE(one.outcomes.one_refused == sampled_cases);
    REQUIRE(one.outcomes.agreed == 0u);
    REQUIRE(one.verdict == agreement::not_exercised);
    REQUIRE_FALSE(every_slot_agreed(judged(answering, alike)));
    REQUIRE(no_residual_was_produced(both));
    REQUIRE(no_residual_was_produced(differently));
    REQUIRE(no_residual_was_produced(one));
}

TEST_CASE("a_slot_no_table_compares_is_absent_from_the_report_and_named_by_the_ledger_instead")
{
    const rigid_motion::capabilities reference     = rigid_motion::baseline();
    const std::array<capability_view, 2> described = rigid_motion::capability_views(reference);
    const std::array<evaluation_view, 2> intact    = rigid_motion::evaluation_views(reference, reference);
    std::vector<slot_evaluation> rows(intact.front().slots().begin(), intact.front().slots().end());

    rows.erase(rows.begin() + static_cast<std::ptrdiff_t>(turned_slot));

    const capability_evaluations<rigid_motion::frame_ops> shortened{"rigid_motion", rows};
    const capability_evaluations<rigid_motion::screw_ops> untouched{"rigid_motion", intact.back().slots()};
    const std::array<evaluation_view, 2> compared{evaluation_view::of(reference.frame, reference.frame, shortened), evaluation_view::of(reference.screw, reference.screw, untouched)};
    const evaluation_report reported = evaluate(compared, recorded_seed, sampled_cases);

    // `every_slot_agreed` speaks only of the rows the report carries, and a slot no table compared
    // has no row: it is true here while nothing at all was measured about `frame.rotate_x`. The
    // ledger below is the only place that absence is reported, so a caller reading the verdict
    // without it reads assent into a comparison that never happened.
    REQUIRE(reported.slots.size() == every_slot() - 1u);
    REQUIRE_FALSE(names(disagreeing_slots(reported), "frame.rotate_x"));
    REQUIRE(every_slot_agreed(reported));
    REQUIRE(unevaluated_slots(described, compared) == std::vector<std::string_view>{"frame.rotate_x"});
}
