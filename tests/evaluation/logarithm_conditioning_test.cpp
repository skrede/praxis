#include "book_logarithm.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/rigid_motion/evaluation.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr std::array<std::string_view, 3> logarithm_rows{"screw.matrix_logarithm_so3", "screw.matrix_logarithm_se3_rp", "screw.matrix_logarithm_se3"};
constexpr std::array<std::string_view, 2> pose_logarithm_rows{"screw.matrix_logarithm_se3_rp", "screw.matrix_logarithm_se3"};

std::size_t differed_in_the_run(const evaluation_report &reported, std::string_view row)
{
    for(const slot_report &slot : reported.slots)
        if(slot.slot == row)
            return slot.outcomes.differed;

    return 0;
}

std::size_t differed_near_the_singular(const std::array<evaluation_view, 2> &compared, std::string_view row)
{
    std::size_t differed = 0;

    for(const evaluation_view &view : compared)
        for(const slot_evaluation &slot : view.slots())
            for(std::size_t index = 0; slot.name == row && index < default_cases_per_slot; ++index)
            {
                case_source drawn = case_source::at_case(default_seed, slot.name, spread::near_singular, index);
                differed += slot.compare(view.first(), view.second(), drawn, slot.allowed).verdict == agreement::differed ? 1u : 0u;
            }

    return differed;
}

template<std::size_t N>
void every_case_of_the_run_differs(const rigid_motion::capabilities &wrong, const std::array<std::string_view, N> &rows)
{
    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, wrong);
    const evaluation_report reported              = evaluate(compared);

    for(std::string_view row : rows)
    {
        INFO(row);
        REQUIRE(differed_in_the_run(reported, row) == default_cases_per_slot);
    }
}

void every_case_of_both_spreads_differs(const rigid_motion::capabilities &wrong)
{
    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, wrong);

    every_case_of_the_run_differs(wrong, logarithm_rows);
    for(std::string_view row : logarithm_rows)
    {
        INFO(row);
        REQUIRE(differed_near_the_singular(compared, row) == default_cases_per_slot);
    }
}

}

TEST_CASE("a_literal_transcription_of_the_modern_robotics_logarithms_agrees_at_the_shipped_seed_and_the_three_after_it")
{
    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const rigid_motion::capabilities book         = fixture::book_logarithms();
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, book);

    for(std::uint64_t seed = default_seed; seed <= default_seed + 3; ++seed)
    {
        const evaluation_report reported = evaluate(compared, seed);

        INFO("seed " << seed);
        for(std::string_view row : logarithm_rows)
        {
            INFO(row);
            CHECK(differed_in_the_run(reported, row) == 0u);
        }
        CHECK(every_slot_agreed(reported));
    }
}

TEST_CASE("a_logarithm_with_its_axis_negated_its_rotation_transposed_or_its_angle_doubled_differs_on_every_case_of_the_run")
{
    every_case_of_the_run_differs(fixture::changed_logarithms<fixture::axis_negated>(), logarithm_rows);
    every_case_of_the_run_differs(fixture::transposed_logarithms(), logarithm_rows);
    every_case_of_the_run_differs(fixture::changed_logarithms<fixture::angle_doubled>(), logarithm_rows);
}

TEST_CASE("a_screw_logarithm_reading_the_position_as_its_linear_half_differs_on_every_case_of_the_run")
{
    every_case_of_the_run_differs(fixture::position_as_linear_half_logarithms(), pose_logarithm_rows);
}

TEST_CASE("an_axis_off_unit_length_or_undefined_differs_on_every_case_of_both_spreads")
{
    every_case_of_both_spreads_differs(fixture::changed_logarithms<fixture::unit_half_doubled>());
    every_case_of_both_spreads_differs(fixture::changed_logarithms<fixture::axis_undefined>());
}
