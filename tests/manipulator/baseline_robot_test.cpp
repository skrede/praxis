#include "three_link_arm.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/robot.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/rigid_motion/baseline/frame.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <numbers>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::fixture;

namespace {

// A rotation and a translation together: an offset carrying only one of the two cannot tell a
// composed inverse from a transposed transform.
transform tool_offset()
{
    return rigid_motion::transformation_matrix_from_rotation_position(rigid_motion::rotate_z(0.3), Eigen::Vector3d(0.1, 0.05, 0.0));
}

transform flange_pose()
{
    return rigid_motion::transformation_matrix_from_rotation_position(rigid_motion::rotate_z(-0.7), Eigen::Vector3d(0.4, -0.2, 0.9));
}

// A pure translation far from any pose the fixture composes: it is the inverse of nothing the cases
// pass, so an answer carrying it can only have come through the substituted slot.
transform marked_pose()
{
    return rigid_motion::transformation_matrix_from_position(Eigen::Vector3d(5.0, 6.0, 7.0));
}

transform marked_inverse(const transform &)
{
    return marked_pose();
}

}

TEST_CASE("composing_a_flange_pose_with_a_tool_offset_and_decomposing_it_returns_the_flange_pose")
{
    const transform flange = flange_pose();
    const transform tool   = tool_pose_from_flange_pose(flange, tool_offset());

    CHECK_FALSE(is_approx_equal(tool, flange));
    CHECK(is_approx_equal(flange_pose_from_tool_pose(rigid_motion::baseline().frame, tool, tool_offset()), flange));
}

TEST_CASE("decomposing_a_tool_pose_against_frame_operations_left_on_their_defaults_returns_the_tool_pose")
{
    const transform flange   = flange_pose();
    const transform tool     = tool_pose_from_flange_pose(flange, tool_offset());
    const transform answered = flange_pose_from_tool_pose(rigid_motion::frame_ops{}, tool, tool_offset());

    CHECK(is_approx_equal(answered, tool));
    CHECK_FALSE(is_approx_equal(answered, flange));
}

TEST_CASE("decomposing_a_tool_pose_applies_the_inverse_bound_into_the_frame_operations_it_is_handed")
{
    const rigid_motion::frame_ops substituted{.inverse = &marked_inverse};
    const transform tool     = tool_pose_from_flange_pose(flange_pose(), tool_offset());
    const transform answered = flange_pose_from_tool_pose(substituted, tool, tool_offset());

    CHECK(is_approx_equal(answered, tool * marked_pose()));
    CHECK_FALSE(is_approx_equal(answered, flange_pose_from_tool_pose(rigid_motion::baseline().frame, tool, tool_offset())));
}

TEST_CASE("the_accessors_return_the_translation_column_and_the_rotation_block_of_a_pose")
{
    const rotation orientation = rigid_motion::rotate_y(std::numbers::pi / 3.0);
    const Eigen::Vector3d at(0.3, -0.8, 1.2);
    const transform pose = rigid_motion::transformation_matrix_from_rotation_position(orientation, at);

    CHECK(position_from_pose(pose).isApprox(at, default_tolerance));
    CHECK(orientation_from_pose(pose).isApprox(orientation, default_tolerance));
}

TEST_CASE("solving_for_a_flange_pose_reaches_it_under_forward_kinematics")
{
    const kinematics solver = make_kinematics(three_link_arm(), baseline().fk, baseline().dk, baseline().ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).value();
    const transform wanted  = solver.fk_solve(posed_arm()).value();
    const joint_vector j0   = arm_configuration(0.3, -1.1, 0.45);

    const expected<joint_vector, refusal> solution = ik_solve_flange_pose(solver, wanted, j0);

    REQUIRE(solution.has_value());
    CHECK_FALSE(is_approx_equal(*solution, j0));
    CHECK(is_approx_equal(solver.fk_solve(*solution).value(), wanted, solved_tolerance));
}

TEST_CASE("solving_for_a_tool_pose_reaches_it_once_the_tool_offset_is_reapplied")
{
    const kinematics solver = make_kinematics(three_link_arm(), baseline().fk, baseline().dk, baseline().ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).value();
    const transform wanted  = tool_pose_from_flange_pose(solver.fk_solve(posed_arm()).value(), tool_offset());
    const joint_vector j0   = arm_configuration(0.3, -1.1, 0.45);

    const expected<joint_vector, refusal> solution = ik_solve_pose(solver, rigid_motion::baseline().frame, wanted, j0, tool_offset());

    REQUIRE(solution.has_value());
    CHECK_FALSE(is_approx_equal(*solution, j0));
    CHECK(is_approx_equal(tool_pose_from_flange_pose(solver.fk_solve(*solution).value(), tool_offset()), wanted, solved_tolerance));
}

// A seed of the wrong length reaches the solver as a vector it cannot difference against the chain,
// so it is refused at the reference's own boundary rather than handed on.
TEST_CASE("a_seed_of_a_size_the_chain_does_not_have_is_refused_rather_than_handed_to_the_solver")
{
    const kinematics solver = make_kinematics(three_link_arm(), baseline().fk, baseline().dk, baseline().ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).value();
    const transform wanted  = solver.fk_solve(posed_arm()).value();
    const joint_vector j0   = joint_vector::Constant(2, 0.3);

    const expected<joint_vector, refusal> from_flange = ik_solve_flange_pose(solver, wanted, j0);
    const expected<joint_vector, refusal> from_tool   = ik_solve_pose(solver, rigid_motion::baseline().frame, wanted, j0, tool_offset());

    REQUIRE_FALSE(from_flange.has_value());
    REQUIRE_FALSE(from_tool.has_value());
    CHECK(from_flange.error() == refusal::unsupported_input);
    CHECK(from_tool.error() == refusal::unsupported_input);
}
