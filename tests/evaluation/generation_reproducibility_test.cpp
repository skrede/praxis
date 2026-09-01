#include "praxis/evaluation/report.h"
#include "praxis/evaluation/tolerance.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/comparators.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <vector>
#include <cstddef>
#include <cstdint>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr std::uint64_t recorded_seed = 0x5EEDu;
constexpr std::size_t sequence_length = 100;
constexpr std::size_t probe_cases     = 64;

double turned_by(double radians)
{
    return radians;
}

double turned_twice(double radians)
{
    return 2.0 * radians;
}

struct probe_ops
{
    double (*turned)(double radians) = &turned_by;
};

constexpr std::array probe_table{
        slot_evaluation{"probe.turn", residual_kind::element_wise, tolerance_of(residual_kind::element_wise),
                        [](const void *first, const void *second, case_source &drawn, const tolerance_pair &allowed) -> case_result
                        {
                            const double radians        = drawn.angle_radians();
                            const Eigen::MatrixXd here  = Eigen::MatrixXd::Constant(1, 1, static_cast<const probe_ops *>(first)->turned(radians));
                            const Eigen::MatrixXd there = Eigen::MatrixXd::Constant(1, 1, static_cast<const probe_ops *>(second)->turned(radians));
                            const residual difference   = element_wise_residual(here, there);

                            return case_result{verdict_of(difference, allowed), difference};
                        }},
};

constexpr capability_evaluations<probe_ops> described_probes{"probe", probe_table};

void one_round(case_source &drawn, std::vector<double> &values)
{
    values.push_back(drawn.angle_radians());
    values.push_back(drawn.pitch());
    values.push_back(drawn.unit_direction().sum());
    values.push_back(drawn.position_metres().sum());
    values.push_back(drawn.euler_triple_radians().sum());
    values.push_back(drawn.angular_part().sum());
    values.push_back(drawn.linear_part().sum());
    values.push_back(drawn.normal_triple().sum());
    values.push_back(drawn.orthonormal_triple().sum());
    values.push_back(drawn.rotation_member().sum());
    values.push_back(drawn.transform_member().sum());
    values.push_back(drawn.skew_symmetric_member().cwiseAbs().sum());
    values.push_back(drawn.unit_twist().sum());
    values.push_back(drawn.twist_member().sum());
    values.push_back(static_cast<double>(drawn.axis_order_index()));
    values.push_back(static_cast<double>(drawn.axis_count()));
}

std::vector<double> sequence(case_source drawn)
{
    std::vector<double> values;

    while(values.size() < sequence_length)
        one_round(drawn, values);

    return values;
}

}

TEST_CASE("two_sources_built_from_the_same_seed_name_and_spread_draw_an_identical_sequence")
{
    const std::vector<double> first  = sequence(case_source::for_slot(recorded_seed, "probe.turn"));
    const std::vector<double> second = sequence(case_source::for_slot(recorded_seed, "probe.turn"));

    REQUIRE(first.size() >= sequence_length);
    REQUIRE(first == second);
}

TEST_CASE("two_sources_built_from_the_same_seed_and_spread_but_different_names_draw_different_sequences")
{
    const std::vector<double> first  = sequence(case_source::for_slot(recorded_seed, "probe.turn"));
    const std::vector<double> second = sequence(case_source::for_slot(recorded_seed, "probe.slide"));

    REQUIRE(first != second);
}

TEST_CASE("two_sources_built_from_the_same_seed_and_name_but_different_spreads_draw_different_sequences")
{
    const std::vector<double> bulk    = sequence(case_source::for_slot(recorded_seed, "probe.turn", spread::bulk));
    const std::vector<double> crowded = sequence(case_source::for_slot(recorded_seed, "probe.turn", spread::near_singular));

    REQUIRE(bulk != crowded);
}

TEST_CASE("a_case_is_redrawn_from_its_index_alone_without_replaying_the_cases_before_it")
{
    const std::vector<double> zeroth = sequence(case_source::at_case(recorded_seed, "probe.turn", spread::bulk, 0));

    REQUIRE(zeroth == sequence(case_source::for_slot(recorded_seed, "probe.turn")));

    for(std::size_t index = 1; index < 8u; ++index)
    {
        const std::vector<double> once  = sequence(case_source::at_case(recorded_seed, "probe.turn", spread::bulk, index));
        const std::vector<double> again = sequence(case_source::at_case(recorded_seed, "probe.turn", spread::bulk, index));

        REQUIRE(once == again);
        REQUIRE(once != zeroth);
    }
}

TEST_CASE("a_source_answers_the_seed_and_the_spread_it_was_built_with")
{
    const case_source built(recorded_seed, spread::near_singular);

    REQUIRE(built.seed() == recorded_seed);
    REQUIRE(built.drawn_from() == spread::near_singular);
    REQUIRE(case_source::for_slot(recorded_seed, "probe.turn").drawn_from() == spread::bulk);
    REQUIRE(case_source::for_slot(recorded_seed, "probe.turn", spread::near_singular).drawn_from() == spread::near_singular);
}

// The worst case a run reports is redrawn from the report's seed and that slot report's case index,
// with nothing else of the run in hand.
TEST_CASE("the_seed_and_the_worst_case_index_a_report_carries_are_enough_to_redraw_the_input")
{
    const probe_ops reference{};
    const probe_ops doubled{&turned_twice};
    const std::array<evaluation_view, 1> compared{evaluation_view::of(reference, doubled, described_probes)};
    const evaluation_report reported = evaluate(compared, recorded_seed, probe_cases);
    const slot_report &slot          = reported.slots.front();

    case_source redrawn = case_source::at_case(reported.seed, slot.slot, spread::bulk, slot.worst_case_index);

    REQUIRE(slot.cases == probe_cases);
    REQUIRE(slot.verdict == agreement::differed);
    REQUIRE(std::fabs(redrawn.angle_radians()) == slot.worst.magnitude);
}
