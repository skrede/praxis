#include "book_screw_maps.h"

#include "praxis/rigid_motion/evaluation.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <string_view>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr std::array<std::uint64_t, 5> seeds{0x5EEDu, 0xC0FFEEu, 0xA11CEu, 0xBEEFu, 0xD1CEu};
constexpr std::size_t cases_per_seed = 10000u;

constexpr std::array<std::string_view, 3> logarithm_rows{"screw.matrix_logarithm_so3", "screw.matrix_logarithm_se3_rp", "screw.matrix_logarithm_se3"};

rigid_motion::capabilities book_maps()
{
    using namespace praxis::tests::book;
    rigid_motion::capabilities book            = rigid_motion::baseline();
    book.screw                                 = rigid_motion::screw_ops{.skew_symmetric                        = &skew_symmetric,
                                                                         .from_skew_symmetric                   = &from_skew_symmetric,
                                                                         .adjoint_matrix_from_rotation_position = &adjoint_matrix_from_rotation_position,
                                                                         .adjoint_matrix_from_transform         = &adjoint_matrix_from_transform,
                                                                         .adjoint_map                           = &adjoint_map,
                                                                         .twist_from_angular_linear             = &twist_from_angular_linear,
                                                                         .twist_from_screw                      = &twist_from_screw,
                                                                         .twist_matrix_from_angular_linear      = &twist_matrix_from_angular_linear,
                                                                         .twist_matrix_from_twist               = &twist_matrix_from_twist,
                                                                         .screw_axis_from_angular_linear        = &screw_axis_from_angular_linear,
                                                                         .screw_axis_from_point_direction_pitch = &screw_axis_from_point_direction_pitch,
                                                                         .matrix_exponential_so3                = &matrix_exponential_so3,
                                                                         .matrix_exponential_se3                = &matrix_exponential_se3,
                                                                         .matrix_exponential_screw              = &matrix_exponential_screw,
                                                                         .matrix_logarithm_so3                  = &matrix_logarithm_so3,
                                                                         .matrix_logarithm_se3_rp               = &matrix_logarithm_se3_rp,
                                                                         .matrix_logarithm_se3                  = &matrix_logarithm_se3};
    book.frame.rotation_matrix_from_axis_angle = &matrix_exponential_so3;

    return book;
}

// Every outcome but agreement and a refusal both sides share.
std::size_t apart(const slot_report &row)
{
    return row.outcomes.differed + row.outcomes.one_refused + row.outcomes.refused_differently + row.outcomes.unusable;
}

template<typename Picked>
void no_case_apart_at_any_seed(Picked picked)
{
    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const rigid_motion::capabilities book         = book_maps();
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, book);

    for(const std::uint64_t seed : seeds)
        for(const slot_report &row : evaluate(compared, seed, cases_per_seed).slots)
            if(picked(row.slot))
            {
                INFO("seed " << seed << ", " << row.slot);
                REQUIRE(row.cases == cases_per_seed);
                REQUIRE(apart(row) == 0u);
            }
}

}

TEST_CASE("a_literal_transcription_of_the_three_logarithms_agrees_on_every_case_at_five_seeds")
{
    no_case_apart_at_any_seed([](std::string_view slot) { return std::ranges::find(logarithm_rows, slot) != logarithm_rows.end(); });
}

TEST_CASE("a_literal_transcription_of_every_screw_map_and_the_axis_angle_rotation_agrees_on_every_case_at_five_seeds")
{
    no_case_apart_at_any_seed([](std::string_view) { return true; });
}
