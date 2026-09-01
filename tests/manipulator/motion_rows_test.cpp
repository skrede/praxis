#include "bent_manipulator.h"
#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/manipulator/evaluation.h"
#include "praxis/manipulator/baseline/motion.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/comparators.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

using namespace praxis;
using namespace praxis::evaluation;
using namespace praxis::manipulator;

namespace {

constexpr std::uint64_t recorded_seed = 0xC0FFEEu;
constexpr std::size_t cases_per_row   = 24u;

// Radians two configurations must stand apart by before this file counts them as different answers,
// far above the step either solve stops within and far beneath a turn onto another branch.
constexpr double branches_apart_radians = 1.0e-3;

constexpr tolerance_pair solved_pose_bound{solved_pose_tolerance_radians, solved_pose_tolerance_metres};

// A second seed, so a binding answering from it lands where the reference does only by reaching the
// same pose and not by taking the same path to it.
joint_vector another_seed(const joint_vector &j0)
{
    return joint_vector(-j0);
}

expected<joint_vector, refusal> pose_from_another_seed(const kinematics &solver, const transform &pose, const joint_vector &j0)
{
    return task_space_pose(solver, pose, another_seed(j0));
}

expected<joint_vector, refusal> screw_from_another_seed(const rigid_motion::screw_ops &screw, const kinematics &solver, const transform &start_pose, const Eigen::Vector3d &w,
                                                        const Eigen::Vector3d &q, double theta_radians, double h, const joint_vector &j0)
{
    return task_space_screw(screw, solver, start_pose, w, q, theta_radians, h, another_seed(j0));
}

expected<joint_vector, refusal> displace_from_another_seed(const kinematics &solver, const transform &start_pose, const Eigen::Vector3d &offset, const rotation &orientation,
                                                           const joint_vector &j0)
{
    return tool_frame_displace(solver, start_pose, offset, orientation, another_seed(j0));
}

constexpr motion_ops answering_from_another_seed{
        .task_space_pose     = &pose_from_another_seed,
        .task_space_screw    = &screw_from_another_seed,
        .tool_frame_displace = &displace_from_another_seed,
};

bool a_motion_row(std::string_view slot)
{
    return slot.substr(0, 7) == "motion.";
}

// Whether the configuration stands at the pose the row asked for, judged at the row's own bound.
bool stands_at(const solve_case &solved, const joint_vector &answer, const transform &asked_for)
{
    const std::optional<transform> reached = reached_by(solved, answer);

    return reached && verdict_of(pose_residual(*reached, asked_for), solved_pose_bound) == agreement::agreed;
}

// How many of the drawn cases the two seeds answered from configurations that are not the same
// value, with both of them standing at the pose the row asked for. A run of none of these would let
// the case below pass over answers that never parted.
std::size_t branches_apart()
{
    std::size_t seen = 0;

    for(std::size_t index = 0; index < cases_per_row; ++index)
    {
        case_source drawn                      = case_source::at_case(recorded_seed, "motion.task_space_pose", spread::bulk, index);
        const std::optional<solve_case> solved = drawn_solve(drawn);
        if(!solved)
            continue;

        const expected<joint_vector, refusal> here  = task_space_pose(solved->solver, solved->reached, solved->seed);
        const expected<joint_vector, refusal> there = pose_from_another_seed(solved->solver, solved->reached, solved->seed);
        if(!here || !there || (*here - *there).cwiseAbs().maxCoeff() <= branches_apart_radians)
            continue;

        seen += stands_at(*solved, *here, solved->reached) && stands_at(*solved, *there, solved->reached) ? 1u : 0u;
    }

    return seen;
}

evaluation_report reported_against(const capabilities &reference, const capabilities &other)
{
    return evaluate(evaluation_views(reference, other), recorded_seed, cases_per_row);
}

}

TEST_CASE("the_reference_answers_every_motion_row_at_the_pose_it_was_asked_for")
{
    fixture::bend_every_row_by({});

    const capabilities reference     = baseline();
    const evaluation_report reported = reported_against(reference, reference);
    std::size_t motion_rows          = 0;

    for(const slot_report &slot : reported.slots)
        if(a_motion_row(slot.slot))
        {
            ++motion_rows;
            INFO("row " << slot.slot);
            REQUIRE(slot.verdict == agreement::agreed);
            REQUIRE(slot.outcomes.agreed > 0u);
            REQUIRE(slot.outcomes.unusable == 0u);
        }

    REQUIRE(motion_rows > 0u);
}

TEST_CASE("two_bindings_reaching_the_pose_asked_for_from_configurations_that_are_not_equal_agree")
{
    fixture::bend_every_row_by({});

    const capabilities reference = baseline();
    capabilities other           = reference;

    other.motion = answering_from_another_seed;

    const evaluation_report reported = reported_against(reference, other);
    std::size_t motion_rows          = 0;

    REQUIRE(branches_apart() > 0u);
    for(const slot_report &slot : reported.slots)
        if(a_motion_row(slot.slot))
        {
            ++motion_rows;
            INFO("row " << slot.slot);
            REQUIRE(slot.verdict == agreement::agreed);
            REQUIRE(slot.outcomes.agreed > 0u);
        }

    REQUIRE(motion_rows > 0u);
}

// Both sides answer the same configuration and it stands at a pose the row did not ask for. A row
// holding one answer against the other would call that agreement; each answer is held against the
// pose that was asked for instead, so the row reports the difference.
TEST_CASE("two_answers_alike_and_both_standing_at_another_pose_are_reported_as_a_difference")
{
    const capabilities reference = baseline();
    capabilities wrong           = reference;

    fixture::bend_every_row_by({0.0, 0.0, 0.0, 0.0, 1.0e-3});

    wrong.motion = fixture::bent_everywhere().motion;

    const evaluation_report reported = reported_against(wrong, wrong);
    std::size_t motion_rows          = 0;

    fixture::bend_every_row_by({});

    for(const slot_report &slot : reported.slots)
        if(a_motion_row(slot.slot))
        {
            ++motion_rows;
            INFO("row " << slot.slot);
            REQUIRE(slot.verdict == agreement::differed);
        }

    REQUIRE(motion_rows > 0u);
}

// A case whose shared inputs could not be built is the harness's own outcome and is attributed to
// neither side: an answer's pose is read through the harness's own forward map, and a map that
// declined leaves nothing to hold against what was asked for.
TEST_CASE("a_case_whose_forward_map_declined_an_answer_is_unusable_and_names_neither_side")
{
    const transform asked_for = transform::Identity();

    REQUIRE(reaching_what_was_asked(std::nullopt, asked_for, asked_for, solved_pose_bound).verdict == agreement::unusable);
    REQUIRE(reaching_what_was_asked(asked_for, std::nullopt, asked_for, solved_pose_bound).verdict == agreement::unusable);
    REQUIRE(reaching_what_was_asked(asked_for, asked_for, asked_for, solved_pose_bound).verdict == agreement::agreed);
}
