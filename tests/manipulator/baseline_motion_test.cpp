#include "three_link_arm.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/robot.h"
#include "praxis/manipulator/baseline/motion.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::fixture;

namespace {

constexpr double turn = 0.3;
constexpr double step = 0.05;

const rigid_motion::screw_ops reference_screw = rigid_motion::baseline().screw;

// The turn reversed, which no correct exponential answers.
transform turned_the_other_way(const screw_axis &s, double theta_radians)
{
    return rigid_motion::matrix_exponential_screw(s, -theta_radians);
}

rigid_motion::screw_ops reversing_the_exponential()
{
    rigid_motion::screw_ops screw  = reference_screw;
    screw.matrix_exponential_screw = &turned_the_other_way;

    return screw;
}

}

TEST_CASE("resolving_a_task_space_pose_reaches_it_under_forward_kinematics")
{
    const kinematics solver = make_kinematics(three_link_arm(), baseline().fk, baseline().dk, baseline().ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).value();
    const transform wanted  = solver.fk_solve(posed_arm()).value();
    const joint_vector j0   = arm_configuration(0.3, -1.1, 0.45);

    const expected<joint_vector, refusal> solution = task_space_pose(solver, wanted, j0);

    REQUIRE(solution.has_value());
    CHECK_FALSE(is_approx_equal(*solution, j0));
    CHECK(is_approx_equal(solver.fk_solve(*solution).value(), wanted, solved_tolerance));
}

TEST_CASE("a_screw_motion_of_zero_angle_returns_the_seed_configuration")
{
    const kinematics solver = make_kinematics(three_link_arm(), baseline().fk, baseline().dk, baseline().ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).value();
    const joint_vector at   = posed_arm();
    const transform start   = solver.fk_solve(at).value();

    const expected<joint_vector, refusal> held = task_space_screw(reference_screw, solver, start, Eigen::Vector3d::UnitZ(), Eigen::Vector3d(0.2, 0.3, 0.0), 0.0, 0.0, at);

    REQUIRE(held.has_value());
    CHECK(is_approx_equal(*held, at));
}

// A zero-pitch screw fixes every point of its axis, so a turn about the axis through the tool origin
// is a pure reorientation of the tool. An axis construction that dropped the point it passes through
// would carry the origin around the world z instead.
TEST_CASE("a_zero_pitch_screw_through_the_tool_origin_reorients_the_tool_and_leaves_it_in_place")
{
    const kinematics solver = make_kinematics(three_link_arm(), baseline().fk, baseline().dk, baseline().ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).value();
    const joint_vector at   = posed_arm();
    const transform start   = solver.fk_solve(at).value();

    const expected<joint_vector, refusal> spun = task_space_screw(reference_screw, solver, start, Eigen::Vector3d::UnitZ(), position_from_pose(start), turn, 0.0, at);
    REQUIRE(spun.has_value());

    const transform reached = solver.fk_solve(*spun).value();
    const rotation turned   = rigid_motion::rotate_z(turn) * orientation_from_pose(start);

    CHECK((position_from_pose(reached) - position_from_pose(start)).norm() < solved_tolerance);
    CHECK(orientation_from_pose(reached).isApprox(turned, solved_tolerance));
}

// The displacement is read in the tool frame, so the origin travels along the tool's own axis; the
// same displacement applied on the left of the start pose would travel along the world's.
TEST_CASE("displacing_the_tool_frame_along_its_own_axis_moves_the_origin_there_and_holds_the_orientation")
{
    const kinematics solver = make_kinematics(three_link_arm(), baseline().fk, baseline().dk, baseline().ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).value();
    const joint_vector at   = posed_arm();
    const transform start   = solver.fk_solve(at).value();

    const expected<joint_vector, refusal> moved = tool_frame_displace(solver, start, Eigen::Vector3d(step, 0.0, 0.0), rotation::Identity(), at);
    REQUIRE(moved.has_value());

    const transform reached = solver.fk_solve(*moved).value();

    const Eigen::Vector3d along_tool = position_from_pose(start) + step * orientation_from_pose(start).col(0);
    const Eigen::Vector3d along_world(position_from_pose(start) + Eigen::Vector3d(step, 0.0, 0.0));

    CHECK((position_from_pose(reached) - along_tool).norm() < solved_tolerance);
    CHECK((along_tool - along_world).norm() > solved_tolerance);
    CHECK(orientation_from_pose(reached).isApprox(orientation_from_pose(start), solved_tolerance));
}

// A screw axis has a direction; the construction answers the zero axis for a zero one, which under
// the exponential is the identity, so an unusable request would be indistinguishable from a turn of
// no angle about a real axis.
TEST_CASE("a_screw_about_no_direction_is_refused_rather_than_resolved_at_the_start_pose")
{
    const kinematics solver = make_kinematics(three_link_arm(), baseline().fk, baseline().dk, baseline().ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).value();
    const joint_vector at   = posed_arm();
    const transform start   = solver.fk_solve(at).value();

    const expected<joint_vector, refusal> nowhere = task_space_screw(reference_screw, solver, start, Eigen::Vector3d::Zero(), position_from_pose(start), turn, 0.0, at);

    REQUIRE_FALSE(nowhere.has_value());
    CHECK(nowhere.error() == refusal::degenerate);
}

TEST_CASE("a_seed_of_a_size_the_chain_does_not_have_is_refused_by_every_motion_resolution")
{
    const kinematics solver = make_kinematics(three_link_arm(), baseline().fk, baseline().dk, baseline().ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).value();
    const transform start   = solver.fk_solve(posed_arm()).value();
    const joint_vector at   = joint_vector::Constant(2, 0.3);

    const expected<joint_vector, refusal> to_pose  = task_space_pose(solver, start, at);
    const expected<joint_vector, refusal> to_screw = task_space_screw(reference_screw, solver, start, Eigen::Vector3d::UnitZ(), position_from_pose(start), turn, 0.0, at);
    const expected<joint_vector, refusal> jogged   = tool_frame_displace(solver, start, Eigen::Vector3d(step, 0.0, 0.0), rotation::Identity(), at);

    REQUIRE_FALSE(to_pose.has_value());
    REQUIRE_FALSE(to_screw.has_value());
    REQUIRE_FALSE(jogged.has_value());
    CHECK(to_pose.error() == refusal::unsupported_input);
    CHECK(to_screw.error() == refusal::unsupported_input);
    CHECK(jogged.error() == refusal::unsupported_input);
}

TEST_CASE("a_screw_motion_answers_through_the_exponential_it_is_handed_rather_than_the_reference_one")
{
    const kinematics solver = make_kinematics(three_link_arm(), baseline().fk, baseline().dk, baseline().ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).value();
    const joint_vector at   = posed_arm();
    const transform start   = solver.fk_solve(at).value();

    const expected<joint_vector, refusal> held      = task_space_screw(reference_screw, solver, start, Eigen::Vector3d::UnitZ(), position_from_pose(start), turn, 0.0, at);
    const expected<joint_vector, refusal> reversing = task_space_screw(reversing_the_exponential(), solver, start, Eigen::Vector3d::UnitZ(), position_from_pose(start), turn, 0.0, at);

    REQUIRE(held.has_value());
    REQUIRE(reversing.has_value());
    CHECK_FALSE(is_approx_equal(*reversing, *held));
}

// The axis construction a defaulted aggregate carries answers a refusal, and the resolution carries
// that refusal out rather than continuing on a fabricated axis.
TEST_CASE("a_screw_motion_over_a_screw_aggregate_left_on_its_defaults_is_refused_rather_than_resolved")
{
    const kinematics solver = make_kinematics(three_link_arm(), baseline().fk, baseline().dk, baseline().ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).value();
    const joint_vector at   = posed_arm();
    const transform start   = solver.fk_solve(at).value();

    const expected<joint_vector, refusal> unbound = task_space_screw(rigid_motion::screw_ops{}, solver, start, Eigen::Vector3d::UnitZ(), position_from_pose(start), turn, 0.0, at);

    REQUIRE_FALSE(unbound.has_value());
    CHECK(unbound.error() == refusal::not_implemented);
}
