#include "book_chain_maps.h"
#include "evaluation_cases.h"

#include "praxis/trajectory/evaluation.h"
#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/comparators.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr std::array<std::uint64_t, 5> seeds{0x5EEDu, 0xC0FFEEu, 0xA11CEu, 0xBEEFu, 0xD1CEu};
constexpr std::size_t cases_per_seed = 10000u;
constexpr std::size_t screw_row      = 1u;
constexpr std::size_t decoupled_row  = 2u;

using path_slot = expected<transform, refusal> (*)(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &start, const transform &end, double s);

// The chapter 3 transcription standing in for the screw and frame operations a path is handed.
rigid_motion::capabilities chapter_three()
{
    rigid_motion::capabilities book           = rigid_motion::baseline();
    book.screw.matrix_exponential_so3         = &tests::book::matrix_exponential_so3;
    book.screw.matrix_exponential_screw       = &tests::book::matrix_exponential_screw;
    book.screw.matrix_logarithm_so3           = &tests::book::matrix_logarithm_so3;
    book.screw.matrix_logarithm_se3           = &tests::book::matrix_logarithm_se3;
    book.frame.inverse                        = &tests::book::inverse_of;
    book.frame.rotation_matrix_from_transform = [](const transform &tf) -> rotation { return tf.block<3, 3>(0, 0); };
    return book;
}

// Only the path view is evaluated: it is the second the aggregate lists.
std::array<slot_report, 2> path_rows(const trajectory::capabilities &other, std::uint64_t seed, std::size_t cases)
{
    const trajectory::capabilities reference = trajectory::baseline();
    const auto views                         = trajectory::evaluation_views(reference, other);
    const evaluation_report reported         = evaluate(std::span(views).subspan(1, 1), seed, cases);

    REQUIRE(reported.slots.at(screw_row).slot == "path.screw");
    REQUIRE(reported.slots.at(decoupled_row).slot == "path.decoupled");
    return {reported.slots.at(screw_row), reported.slots.at(decoupled_row)};
}

// The cases whose literal answer the shared pose residual reads as no rotation at all.
std::size_t outside_the_shared_membership(std::string_view row, path_slot literal, path_slot reference)
{
    std::size_t outside                      = 0;
    const rigid_motion::capabilities book    = chapter_three();
    const rigid_motion::capabilities held_to = rigid_motion::baseline();
    for(const std::uint64_t seed : seeds)
        for(std::size_t index = 0; index < cases_per_seed; ++index)
        {
            case_source drawn                       = case_source::at_case(seed, row, spread::bulk, index);
            const trajectory::pose_path_case sample = trajectory::drawn_pose_path_case(drawn);
            const transform held                    = literal(book.screw, book.frame, sample.start, sample.end, sample.s).value();
            const transform against                 = reference(held_to.screw, held_to.frame, sample.start, sample.end, sample.s).value();
            outside += std::isinf(pose_residual(held, against).magnitude) ? 1u : 0u;
        }

    return outside;
}

expected<transform, refusal> screw_premultiplied(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &start, const transform &end, double s)
{
    const auto logged = screw.matrix_logarithm_se3(frames.inverse(start) * end).value();
    return transform(screw.matrix_exponential_screw(logged.first, logged.second * s) * start);
}

expected<transform, refusal> screw_relative_motion_reversed(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &start, const transform &end,
                                                            double s)
{
    const auto logged = screw.matrix_logarithm_se3(frames.inverse(end) * start).value();
    return transform(start * screw.matrix_exponential_screw(logged.first, logged.second * s));
}

// The literal paths over a rigid-motion module of their own, whose exponentials turn a radian
// further, rather than over the operations they are handed.
expected<transform, refusal> screw_on_its_own_operations(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const transform &start, const transform &end, double s)
{
    rigid_motion::capabilities own     = rigid_motion::baseline();
    own.screw.matrix_exponential_screw = [](const screw_axis &axis, double theta) { return rigid_motion::matrix_exponential_screw(axis, theta + 1.0); };
    return tests::book::screw_path(own.screw, own.frame, start, end, s);
}

expected<transform, refusal> decoupled_on_its_own_operations(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const transform &start, const transform &end, double s)
{
    rigid_motion::capabilities own   = rigid_motion::baseline();
    own.screw.matrix_exponential_so3 = [](const Eigen::Vector3d &w, double theta) { return rigid_motion::matrix_exponential_so3(w, theta + 1.0); };
    return tests::book::decoupled_path(own.screw, own.frame, start, end, s);
}

enum class decoupled_slip : std::uint8_t
{
    rotation_premultiplied,
    relative_rotation_reversed,
    start_not_subtracted,
};

template<decoupled_slip slip>
expected<transform, refusal> slipped_decoupled(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &start, const transform &end, double s)
{
    const rotation r_start  = frames.rotation_matrix_from_transform(start);
    const rotation r_end    = frames.rotation_matrix_from_transform(end);
    const rotation relative = slip == decoupled_slip::relative_rotation_reversed ? rotation(r_end.transpose() * r_start) : rotation(r_start.transpose() * r_end);
    const auto logged       = screw.matrix_logarithm_so3(relative).value();
    const rotation turned   = screw.matrix_exponential_so3(logged.first, logged.second * s);
    const Eigen::Vector3d p = start.block<3, 1>(0, 3);
    const Eigen::Vector3d q = end.block<3, 1>(0, 3);
    transform x             = transform::Identity();
    x.block<3, 3>(0, 0)     = slip == decoupled_slip::rotation_premultiplied ? rotation(turned * r_start) : rotation(r_start * turned);
    x.block<3, 1>(0, 3)     = slip == decoupled_slip::start_not_subtracted ? Eigen::Vector3d(p + s * q) : Eigen::Vector3d(p + s * (q - p));
    return x;
}

void every_case_of_the_run_differs(path_slot screw, path_slot decoupled, std::size_t row)
{
    trajectory::capabilities wrong = trajectory::baseline();
    wrong.path.screw               = screw;
    wrong.path.decoupled           = decoupled;

    REQUIRE(path_rows(wrong, default_seed, default_cases_per_slot).at(row == screw_row ? 0u : 1u).outcomes.differed == default_cases_per_slot);
}

}

TEST_CASE("a_literal_transcription_of_both_task_space_paths_agrees_on_every_case_at_five_seeds")
{
    const trajectory::capabilities book = tests::book::path_maps();

    for(const std::uint64_t seed : seeds)
        for(const slot_report &row : path_rows(book, seed, cases_per_seed))
        {
            INFO("seed " << seed << ", " << row.slot);
            REQUIRE(row.cases == cases_per_seed);
            REQUIRE(row.outcomes.agreed == cases_per_seed);
        }
}

TEST_CASE("the_literal_paths_over_the_operations_they_are_handed_agree_on_every_case_at_the_default_seed_and_forty_after_it")
{
    const trajectory::capabilities book = tests::book::path_maps();

    for(std::uint64_t seed = default_seed; seed <= default_seed + 40u; ++seed)
        for(const slot_report &row : path_rows(book, seed, default_cases_per_slot))
        {
            INFO("seed " << seed << ", " << row.slot);
            REQUIRE(row.outcomes.agreed == default_cases_per_slot);
        }
}

TEST_CASE("a_literal_path_over_a_rigid_motion_module_of_its_own_differs_on_every_case_of_the_run")
{
    const trajectory::capabilities reference = trajectory::baseline();

    every_case_of_the_run_differs(&screw_on_its_own_operations, reference.path.decoupled, screw_row);
    every_case_of_the_run_differs(reference.path.screw, &decoupled_on_its_own_operations, decoupled_row);
}

TEST_CASE("the_literal_paths_answer_rotations_the_shared_membership_test_refuses_on_cases_the_path_rows_draw")
{
    const trajectory::capabilities reference = trajectory::baseline();

    REQUIRE(outside_the_shared_membership("path.screw", &tests::book::screw_path, reference.path.screw) > 0u);
    REQUIRE(outside_the_shared_membership("path.decoupled", &tests::book::decoupled_path, reference.path.decoupled) > 0u);
}

TEST_CASE("a_path_premultiplied_or_reversed_or_its_start_left_unsubtracted_differs_on_every_case_of_the_run")
{
    const trajectory::capabilities reference = trajectory::baseline();

    every_case_of_the_run_differs(&screw_premultiplied, reference.path.decoupled, screw_row);
    every_case_of_the_run_differs(&screw_relative_motion_reversed, reference.path.decoupled, screw_row);
    every_case_of_the_run_differs(reference.path.screw, &slipped_decoupled<decoupled_slip::rotation_premultiplied>, decoupled_row);
    every_case_of_the_run_differs(reference.path.screw, &slipped_decoupled<decoupled_slip::relative_rotation_reversed>, decoupled_row);
    every_case_of_the_run_differs(reference.path.screw, &slipped_decoupled<decoupled_slip::start_not_subtracted>, decoupled_row);
}
