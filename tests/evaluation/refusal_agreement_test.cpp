#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/comparators.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <cstdint>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr std::uint64_t recorded_seed = 0x5EEDu;

using answer = expected<Eigen::Vector3d, refusal>;

struct probe_ops
{
    answer (*measured)(double radians);
};

answer measured(double radians)
{
    return Eigen::Vector3d(radians, 0.0, 0.0);
}

answer measured_apart(double radians)
{
    return Eigen::Vector3d(radians + 1.0, 0.0, 0.0);
}

answer declines_a_positive_angle(double radians)
{
    if(radians > 0.0)
        return unexpected(refusal::degenerate);

    return Eigen::Vector3d(radians, 0.0, 0.0);
}

answer declines_a_positive_angle_and_bends_the_rest(double radians)
{
    if(radians > 0.0)
        return unexpected(refusal::degenerate);

    return Eigen::Vector3d(radians + 1.0, 0.0, 0.0);
}

answer declines_everything(double)
{
    return unexpected(refusal::not_implemented);
}

residual compared(const Eigen::Vector3d &first, const Eigen::Vector3d &second)
{
    return element_wise_residual(first, second);
}

constexpr std::array probe_table{
        slot_evaluation{"probe.measured", residual_kind::element_wise, tolerance_of(residual_kind::element_wise),
                        [](const void *first, const void *second, case_source &drawn, const tolerance_pair &allowed) -> case_result
                        {
                            const double radians = drawn.angle_radians();

                            return agreed_or_refused(static_cast<const probe_ops *>(first)->measured(radians), static_cast<const probe_ops *>(second)->measured(radians), &compared,
                                                     allowed);
                        }},
};

constexpr capability_evaluations<probe_ops> probed{"probe", probe_table};

agreement folded(const probe_ops &first, const probe_ops &second)
{
    const std::array<evaluation_view, 1> pair{evaluation_view::of(first, second, probed)};

    return evaluate(pair, recorded_seed, 64).slots.front().verdict;
}

tolerance_pair allowed()
{
    return tolerance_of(residual_kind::element_wise);
}

}

TEST_CASE("two_sides_that_both_answered_are_judged_on_the_residual_between_them")
{
    const case_result alike = agreed_or_refused(measured(0.5), measured(0.5), &compared, allowed());
    const case_result apart = agreed_or_refused(measured(0.5), measured_apart(0.5), &compared, allowed());

    REQUIRE(alike.verdict == agreement::agreed);
    REQUIRE(alike.difference.magnitude == 0.0);
    REQUIRE(apart.verdict == agreement::differed);
    REQUIRE(apart.difference.magnitude == 1.0);
}

TEST_CASE("two_sides_that_declined_the_same_input_for_the_same_reason_agree_about_that_input")
{
    const case_result seen = agreed_or_refused(declines_everything(0.5), declines_everything(0.5), &compared, allowed());

    REQUIRE(seen.verdict == agreement::both_refused);
    REQUIRE(seen.difference.magnitude == 0.0);
    REQUIRE(seen.difference.linear_error_metres == 0.0);
}

TEST_CASE("two_sides_that_declined_for_different_reasons_are_an_outcome_of_their_own")
{
    const case_result seen = agreed_or_refused(declines_everything(0.5), declines_a_positive_angle(0.5), &compared, allowed());

    REQUIRE(seen.verdict == agreement::refused_differently);
    REQUIRE(seen.difference.magnitude == 0.0);
}

TEST_CASE("exactly_one_side_declining_is_neither_agreement_nor_a_residual")
{
    const case_result declined_second = agreed_or_refused(measured(0.5), declines_everything(0.5), &compared, allowed());
    const case_result declined_first  = agreed_or_refused(declines_everything(0.5), measured(0.5), &compared, allowed());

    REQUIRE(declined_second.verdict == agreement::one_refused);
    REQUIRE(declined_first.verdict == agreement::one_refused);
    REQUIRE(declined_second.difference.magnitude == 0.0);
    REQUIRE(declined_first.difference.magnitude == 0.0);
}

TEST_CASE("a_run_where_every_case_had_both_sides_decline_alike_folds_to_both_refused")
{
    const probe_ops reference{&declines_everything};
    const probe_ops subject{&declines_everything};

    REQUIRE(folded(reference, subject) == agreement::both_refused);
}

TEST_CASE("a_run_mixing_agreement_with_a_one_sided_refusal_is_not_folded_by_the_one_sided_case")
{
    const probe_ops reference{&measured};
    const probe_ops subject{&declines_a_positive_angle};

    REQUIRE(folded(reference, reference) == agreement::agreed);
    REQUIRE(folded(reference, subject) == agreement::agreed);
}

TEST_CASE("a_run_mixing_a_disagreement_with_a_one_sided_refusal_folds_to_the_disagreement")
{
    const probe_ops reference{&measured};
    const probe_ops subject{&declines_a_positive_angle_and_bends_the_rest};

    REQUIRE(folded(reference, subject) == agreement::differed);
}
