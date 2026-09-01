#include "solve_rows.h"
#include "bent_solver.h"
#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/robot.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/comparators.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <vector>
#include <cstddef>
#include <optional>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;
using namespace praxis::evaluation;

namespace {

// The margins the two bounds stand above the worst residual a converged solve can leave: the angular
// half is bounded by the solver's own criterion, and the linear half of a row measured at the tool
// picks up the criterion's angular half over the lever arm the drawn offset stands at.
constexpr double angular_margin = 5.0;
constexpr double tool_margin    = 2.0;

// One drawn case of a row measured at the tool: the request, the offset the row was handed, and the
// pose the reference's own solve reached when asked for it.
struct solved_at_the_tool
{
    residual against_the_request;
    double lever_arm_metres;
};

std::optional<solved_at_the_tool> reference_solve(std::size_t index)
{
    case_source drawn             = case_source::at_case(recorded_seed, "robot.ik_solve_pose", spread::bulk, index);
    const evaluation_case example = drawn_case(drawn);
    const joint_vector seed       = drawn_joints(drawn, example.chain.joint_count());
    const transform offset        = drawn.transform_member();
    const auto composed           = kinematics::compose(example.chain, baseline().fk, baseline().dk, baseline().ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame);
    if(!composed)
        return std::nullopt;

    const expected<transform, refusal> flange = composed->fk_solve(example.joints);
    if(!flange)
        return std::nullopt;

    const transform tool                     = *flange * offset;
    const expected<joint_vector, refusal> at = ik_solve_pose(*composed, rigid_motion::baseline().frame, tool, seed, offset);
    if(!at)
        return std::nullopt;

    const expected<transform, refusal> reached = composed->fk_solve(*at);
    if(!reached)
        return std::nullopt;

    return solved_at_the_tool{pose_residual(transform(*reached * offset), tool), offset.block<3, 1>(0, 3).norm()};
}

}

// The angular half of a converged solve is the criterion itself; the linear half of a row measured at
// the tool is the criterion's linear half plus its angular half turned through the offset's lever arm.
TEST_CASE("the_reference_solve_lands_within_the_criterion_the_solver_publishes_and_the_lever_arm_it_is_read_at")
{
    const solver_parameters criterion;
    double worst_allowed = 0.0;
    std::size_t solved   = 0;

    for(std::size_t index = 0; index < cases_per_row; ++index)
    {
        const std::optional<solved_at_the_tool> seen = reference_solve(index);
        if(!seen)
            continue;

        const double allowed_here = criterion.position_tol + criterion.orientation_tol * seen->lever_arm_metres;

        INFO("case " << index);
        REQUIRE(seen->against_the_request.magnitude <= criterion.orientation_tol);
        REQUIRE(seen->against_the_request.linear_error_metres <= allowed_here);

        worst_allowed = std::max(worst_allowed, allowed_here);
        ++solved;
    }

    REQUIRE(solved > 0u);
    REQUIRE(criterion.orientation_tol * angular_margin <= solved_pose_tolerance_radians);
    REQUIRE(worst_allowed * tool_margin <= solved_pose_tolerance_metres);
}

// The displacement a decade above the bound and the one a decade beneath it, written out rather than
// derived from the bounds, so a bound raised onto the first or lowered onto the second fails here.
// Every case the two sides both answered is reported as a difference at the first and as agreement
// at the second, the solve's own convergence residual standing a further decade beneath the bound.
constexpr double above_the_bound   = 1.0e-5;
constexpr double beneath_the_bound = 1.0e-7;

TEST_CASE("each_solve_row_reports_a_pose_a_decade_above_the_bound_it_carries_and_leaves_one_a_decade_beneath")
{
    const inverse_kinematics_ops astray  = chain_bound_to(&solves_for_a_displaced_target);
    const robot_ops astray_at_both_poses = robot_bound_to(&solve_pose_for_a_displaced_target, &solve_flange_pose_for_a_displaced_target);
    const capabilities reference         = baseline();

    for(const bool angular : {true, false})
        for(const bool above : {true, false})
        {
            const double displaced = above ? above_the_bound : beneath_the_bound;
            off_target_radians     = angular ? displaced : 0.0;
            off_target_metres      = angular ? 0.0 : displaced;

            for(const solve_row &under_test : the_solve_rows(reference.ik, astray, reference.robot, astray_at_both_poses))
            {
                INFO("row " << under_test.row.name << ", angular " << angular << ", above " << above);
                const std::vector<agreement> seen = over_the_run(under_test.row, under_test.first, under_test.second);

                REQUIRE(how_many(seen, above ? agreement::differed : agreement::agreed) > 0u);
                REQUIRE(how_many(seen, above ? agreement::agreed : agreement::differed) == 0u);
            }
        }

    off_target_radians = 0.0;
    off_target_metres  = 0.0;
}
