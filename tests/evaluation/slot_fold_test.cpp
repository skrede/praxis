#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

using namespace praxis::evaluation;

namespace {

constexpr std::uint64_t recorded_seed    = 0x5EEDu;
constexpr std::string_view scripted_slot = "script.answers";

// A binding whose outcome for every case is written out in advance, so a fold is driven over a known
// sequence rather than over whatever a pair of implementations happens to produce. `reached` counts
// the cases the comparator has been asked for; `evaluate` asks for them in index order.
struct script
{
    std::span<const agreement> answers;
    mutable std::size_t reached;
};

case_result answered(const void *first, const void *, case_source &, const tolerance_pair &)
{
    const script &written   = *static_cast<const script *>(first);
    const agreement outcome = written.answers[written.reached++];

    if(outcome == agreement::differed)
        return case_result{outcome, residual{residual_kind::element_wise, 1.0, 0.0}};

    return case_result{outcome, residual{}};
}

constexpr std::array script_table{
        slot_evaluation{scripted_slot, residual_kind::element_wise, tolerance_of(residual_kind::element_wise), &answered},
};

constexpr capability_evaluations<script> scripted{"script", script_table};

// The same row declaring how many cases its bound was measured over, so a run longer than that is
// asked for a verdict the bound does not carry.
constexpr std::size_t measured_to_cases = 3u;

constexpr std::array measured_row_table{
        slot_evaluation{scripted_slot, residual_kind::element_wise, tolerance_of(residual_kind::element_wise), &answered, measured_to_cases},
};

constexpr capability_evaluations<script> measured_row{"script", measured_row_table};

evaluation_report over(const script &written)
{
    const std::array<evaluation_view, 1> pair{evaluation_view::of(written, written, scripted)};

    return evaluate(pair, recorded_seed, written.answers.size());
}

evaluation_report over_a_measured_bound(const script &written)
{
    const std::array<evaluation_view, 1> pair{evaluation_view::of(written, written, measured_row)};

    return evaluate(pair, recorded_seed, written.answers.size());
}

slot_report folded(std::span<const agreement> answers)
{
    const script written{answers, 0};

    return over(written).slots.front();
}

std::array<agreement, 4> three_agreeing_cases_and(agreement one)
{
    return {agreement::agreed, one, agreement::agreed, agreement::agreed};
}

std::size_t summed(const outcome_counts &counted)
{
    return counted.agreed + counted.differed + counted.one_refused + counted.both_refused + counted.refused_differently + counted.unusable;
}

}

TEST_CASE("the_outcomes_a_case_can_reach_hold_the_values_they_were_published_with")
{
    REQUIRE(static_cast<std::uint8_t>(agreement::not_exercised) == 0u);
    REQUIRE(static_cast<std::uint8_t>(agreement::agreed) == 1u);
    REQUIRE(static_cast<std::uint8_t>(agreement::differed) == 2u);
    REQUIRE(static_cast<std::uint8_t>(agreement::both_refused) == 3u);
    REQUIRE(static_cast<std::uint8_t>(agreement::refused_differently) == 4u);
    REQUIRE(static_cast<std::uint8_t>(agreement::one_refused) == 5u);
    REQUIRE(static_cast<std::uint8_t>(agreement::unusable) == 6u);
    REQUIRE(static_cast<std::uint8_t>(agreement::beyond_measurement) == 7u);
}

TEST_CASE("one_case_the_harness_could_not_set_up_folds_the_whole_slot_and_none_leaves_it_alone")
{
    const std::array unusable = three_agreeing_cases_and(agreement::unusable);
    const std::array intact   = three_agreeing_cases_and(agreement::agreed);
    const script written{unusable, 0};
    const evaluation_report run = over(written);

    REQUIRE(run.slots.front().verdict == agreement::unusable);
    REQUIRE(run.slots.front().outcomes.unusable == 1u);
    REQUIRE_FALSE(every_slot_agreed(run));
    REQUIRE(disagreeing_slots(run) == std::vector<std::string_view>{scripted_slot});
    REQUIRE(folded(intact).verdict == agreement::agreed);
    REQUIRE(folded(intact).outcomes.unusable == 0u);
}

TEST_CASE("an_unusable_case_outranks_every_other_outcome_a_case_can_reach")
{
    const std::array<agreement, 2> against_a_difference{agreement::differed, agreement::unusable};
    const std::array<agreement, 2> against_a_refusal{agreement::refused_differently, agreement::unusable};
    const std::array<agreement, 2> against_a_one_sided{agreement::one_refused, agreement::unusable};

    REQUIRE(folded(against_a_difference).verdict == agreement::unusable);
    REQUIRE(folded(against_a_refusal).verdict == agreement::unusable);
    REQUIRE(folded(against_a_one_sided).verdict == agreement::unusable);
}

TEST_CASE("a_one_sided_case_does_not_fold_a_row_whose_answered_cases_all_agreed")
{
    const std::array answers = three_agreeing_cases_and(agreement::one_refused);
    const slot_report row    = folded(answers);

    REQUIRE(row.verdict == agreement::agreed);
    REQUIRE(row.outcomes.agreed == 3u);
    REQUIRE(row.outcomes.one_refused == 1u);
}

TEST_CASE("a_row_no_case_of_which_had_both_sides_answer_is_never_a_pass")
{
    const std::array<agreement, 3> every_case_one_sided{agreement::one_refused, agreement::one_refused, agreement::one_refused};
    const script written{every_case_one_sided, 0};
    const evaluation_report run = over(written);

    REQUIRE(run.slots.front().verdict == agreement::not_exercised);
    REQUIRE(run.slots.front().outcomes.one_refused == 3u);
    REQUIRE_FALSE(every_slot_agreed(run));
    REQUIRE(disagreeing_slots(run) == std::vector<std::string_view>{scripted_slot});
}

TEST_CASE("a_difference_outranks_a_one_sided_case_and_a_refusal_the_two_sides_gave_apart")
{
    const std::array<agreement, 2> against_a_one_sided{agreement::differed, agreement::one_refused};
    const std::array<agreement, 2> against_a_refusal{agreement::differed, agreement::refused_differently};

    REQUIRE(folded(against_a_one_sided).verdict == agreement::differed);
    REQUIRE(folded(against_a_refusal).verdict == agreement::differed);
}

TEST_CASE("only_a_row_every_case_of_which_declined_alike_folds_to_a_decline_by_both_sides")
{
    const std::array<agreement, 3> every_case{agreement::both_refused, agreement::both_refused, agreement::both_refused};
    const std::array<agreement, 2> beside_agreement{agreement::agreed, agreement::both_refused};
    const std::array<agreement, 2> beside_a_refusal_apart{agreement::both_refused, agreement::refused_differently};

    REQUIRE(folded(every_case).verdict == agreement::both_refused);
    REQUIRE(folded(beside_agreement).verdict == agreement::agreed);
    REQUIRE(folded(beside_a_refusal_apart).verdict == agreement::refused_differently);
}

TEST_CASE("a_run_of_no_cases_at_all_is_reported_as_not_exercised_with_every_count_at_zero")
{
    const slot_report row = folded(std::span<const agreement>{});

    REQUIRE(row.verdict == agreement::not_exercised);
    REQUIRE(row.cases == 0u);
    REQUIRE(summed(row.outcomes) == 0u);
}

TEST_CASE("a_row_carries_how_many_of_its_cases_reached_each_outcome_and_the_counts_sum_to_the_cases")
{
    const std::array<agreement, 6> one_of_each{agreement::agreed,  agreement::differed, agreement::one_refused, agreement::both_refused, agreement::refused_differently,
                                               agreement::unusable};
    const slot_report row = folded(one_of_each);

    REQUIRE(row.cases == 6u);
    REQUIRE(row.outcomes.agreed == 1u);
    REQUIRE(row.outcomes.differed == 1u);
    REQUIRE(row.outcomes.one_refused == 1u);
    REQUIRE(row.outcomes.both_refused == 1u);
    REQUIRE(row.outcomes.refused_differently == 1u);
    REQUIRE(row.outcomes.unusable == 1u);
    REQUIRE(summed(row.outcomes) == row.cases);
}

TEST_CASE("two_rows_that_both_agreed_are_told_apart_by_how_many_of_their_cases_agreed")
{
    const std::array<agreement, 2> one{agreement::agreed, agreement::one_refused};
    const std::array<agreement, 5> five{agreement::agreed, agreement::agreed, agreement::agreed, agreement::agreed, agreement::agreed};

    REQUIRE(folded(one).verdict == agreement::agreed);
    REQUIRE(folded(five).verdict == agreement::agreed);
    REQUIRE(folded(one).outcomes.agreed == 1u);
    REQUIRE(folded(five).outcomes.agreed == 5u);
}

TEST_CASE("no_row_reports_a_difference_over_a_residual_of_nothing")
{
    constexpr std::array kinds{residual_kind::element_wise, residual_kind::geodesic, residual_kind::pose, residual_kind::axis_up_to_sign, residual_kind::log_up_to_branch};
    const std::array<agreement, 2> one_differing{agreement::differed, agreement::agreed};
    const slot_report row = folded(one_differing);

    for(residual_kind kind : kinds)
        REQUIRE(verdict_of(residual{kind, 0.0, 0.0}, tolerance_of(kind)) != agreement::differed);

    REQUIRE(row.verdict == agreement::differed);
    REQUIRE(row.worst.magnitude > 0.0);
}

TEST_CASE("a_row_asked_for_more_cases_than_its_bound_was_measured_over_reports_that_and_no_verdict_of_its_own")
{
    const std::array<agreement, 3> up_to_it{agreement::agreed, agreement::agreed, agreement::agreed};
    const std::array<agreement, 4> past_it{agreement::agreed, agreement::agreed, agreement::agreed, agreement::agreed};
    const script up_to{up_to_it, 0};
    const script past{past_it, 0};
    const evaluation_report beyond = over_a_measured_bound(past);

    REQUIRE(over_a_measured_bound(up_to).slots.front().verdict == agreement::agreed);
    REQUIRE(beyond.slots.front().verdict == agreement::beyond_measurement);
    REQUIRE(beyond.slots.front().outcomes.agreed == 4u);
    REQUIRE(beyond.slots.front().cases == 4u);
    REQUIRE_FALSE(every_slot_agreed(beyond));
    REQUIRE(disagreeing_slots(beyond) == std::vector<std::string_view>{scripted_slot});
}

TEST_CASE("a_row_declaring_no_measured_number_of_cases_carries_its_verdict_over_a_run_of_any_length")
{
    const std::array<agreement, 4> longer_than_the_measured_row{agreement::agreed, agreement::agreed, agreement::agreed, agreement::agreed};

    REQUIRE(folded(longer_than_the_measured_row).verdict == agreement::agreed);
}
