#include "solve_rows.h"
#include "bent_solver.h"
#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <vector>
#include <cstddef>
#include <string_view>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;
using namespace praxis::evaluation;

namespace {

// Plainly wrong: three decades above the bound the three rows are judged at.
constexpr double a_long_way_off = 1.0e-3;

}

TEST_CASE("two_solves_reaching_the_same_pose_other_than_the_one_asked_for_differ_on_every_row_that_runs_one")
{
    off_target_radians                   = a_long_way_off;
    off_target_metres                    = a_long_way_off;
    const inverse_kinematics_ops astray  = chain_bound_to(&solves_for_a_displaced_target);
    const robot_ops astray_at_both_poses = robot_bound_to(&solve_pose_for_a_displaced_target, &solve_flange_pose_for_a_displaced_target);

    for(const solve_row &under_test : the_solve_rows(astray, astray, astray_at_both_poses, astray_at_both_poses))
    {
        INFO("row " << under_test.row.name);
        const std::vector<agreement> seen = over_the_run(under_test.row, under_test.first, under_test.second);

        REQUIRE(how_many(seen, agreement::differed) > 0u);
        REQUIRE(how_many(seen, agreement::agreed) == 0u);
    }

    off_target_radians = 0.0;
    off_target_metres  = 0.0;
}

TEST_CASE("two_solves_reaching_the_pose_that_was_asked_for_agree_whatever_configurations_they_reached_it_from")
{
    const capabilities reference           = baseline();
    const inverse_kinematics_ops reflected = chain_bound_to(&solves_from_a_reflected_seed);
    const robot_ops reflected_at_both      = robot_bound_to(&solve_pose_from_a_reflected_seed, &solve_flange_pose_from_a_reflected_seed);

    for(const solve_row &under_test : the_solve_rows(reference.ik, reflected, reference.robot, reflected_at_both))
    {
        INFO("row " << under_test.row.name);
        const std::vector<agreement> seen = over_the_run(under_test.row, under_test.first, under_test.second);

        REQUIRE(how_many(seen, agreement::agreed) > 0u);
        REQUIRE(how_many(seen, agreement::differed) == 0u);
    }
}

// The row above is only worth its name where the two branches it agrees over are apart, which the
// comparison itself does not report. The same draw is replayed here and the two configurations held
// against each other.
TEST_CASE("a_reflected_seed_reaches_the_requested_pose_from_a_configuration_that_is_not_the_reference_one")
{
    const capabilities reference = baseline();
    std::size_t apart            = 0;

    for(std::size_t index = 0; index < cases_per_row; ++index)
    {
        case_source drawn             = case_source::at_case(recorded_seed, "ik.inverse_kinematics", spread::bulk, index);
        const evaluation_case example = drawn_case(drawn);
        const joint_vector seed       = drawn_joints(drawn, example.chain.joint_count());
        const auto composed           = kinematics::compose(example.chain, reference.fk, reference.dk, reference.ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame);
        if(!composed)
            continue;

        const expected<transform, refusal> target = composed->fk_solve(example.joints);
        if(!target)
            continue;

        ik_result here;
        ik_result there;
        const expected<void, refusal> held  = inverse_kinematics(reference.fk, reference.dk, example.chain, *target, seed, solver_parameters(), here);
        const expected<void, refusal> other = solves_from_a_reflected_seed(reference.fk, reference.dk, example.chain, *target, seed, solver_parameters(), there);
        if(!held || !other || here.solutions.empty() || there.solutions.empty())
            continue;

        apart += (here.solutions.front() - there.solutions.front()).cwiseAbs().maxCoeff() > solved_pose_tolerance_radians ? 1u : 0u;
    }

    REQUIRE(apart > 0u);
}

TEST_CASE("an_answer_the_shared_forward_map_cannot_be_taken_over_leaves_the_case_unusable_and_names_neither_side")
{
    const capabilities reference       = baseline();
    const inverse_kinematics_ops wide  = chain_bound_to(&answers_one_joint_too_many);
    const robot_ops wide_at_both_poses = robot_bound_to(&answers_one_joint_too_many_at_the_tool, &answers_one_joint_too_many_at_the_flange);

    for(const solve_row &under_test : the_solve_rows(reference.ik, wide, reference.robot, wide_at_both_poses))
    {
        INFO("row " << under_test.row.name);
        const std::vector<agreement> seen = over_the_run(under_test.row, under_test.first, under_test.second);

        REQUIRE(how_many(seen, agreement::unusable) > 0u);
        REQUIRE(how_many(seen, agreement::differed) == 0u);
        REQUIRE(how_many(seen, agreement::agreed) == 0u);
    }
}

TEST_CASE("the_one_place_a_shared_input_failure_becomes_a_case_reports_the_harnesss_own_outcome_and_no_number")
{
    for(const residual_kind kind : {residual_kind::pose, residual_kind::element_wise, residual_kind::geodesic})
    {
        const case_result seen = unusable(kind);

        REQUIRE(seen.verdict == agreement::unusable);
        REQUIRE(seen.difference.kind == kind);
        REQUIRE(seen.difference.magnitude == 0.0);
        REQUIRE(seen.difference.linear_error_metres == 0.0);
    }
}

TEST_CASE("the_two_robot_solve_rows_report_a_refusal_exactly_as_the_facility_always_has")
{
    const capabilities reference = baseline();
    const robot_ops declining    = robot_bound_to(&declines_at_the_tool, &declines_at_the_flange);
    const robot_ops otherwise    = robot_bound_to(&declines_at_the_tool_for_another_reason, &declines_at_the_flange_for_another_reason);

    for(const std::string_view name : {std::string_view("robot.ik_solve_pose"), std::string_view("robot.ik_solve_flange_pose")})
    {
        const slot_evaluation &row           = named(robot_evaluations().slots, name);
        const std::vector<agreement> alike   = over_the_run(row, &declining, &declining);
        const std::vector<agreement> apart   = over_the_run(row, &declining, &otherwise);
        const std::vector<agreement> one_way = over_the_run(row, &reference.robot, &declining);

        INFO("row " << name);
        REQUIRE(how_many(alike, agreement::both_refused) == cases_per_row);
        REQUIRE(how_many(apart, agreement::refused_differently) == cases_per_row);
        REQUIRE(how_many(one_way, agreement::one_refused) + how_many(one_way, agreement::both_refused) == cases_per_row);
        REQUIRE(how_many(one_way, agreement::one_refused) > 0u);
    }
}
