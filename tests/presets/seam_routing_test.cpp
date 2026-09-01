#include "three_link_arm.h"
#include "substituted_rigid_motion.h"

#include "praxis/manipulator/robot.h"
#include "praxis/manipulator/motion.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <vector>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

constexpr double turn = -0.35;

// How far apart in any one joint two configurations must stand before this file counts them as
// different answers, far above the step the solve stops within.
constexpr double apart_by_radians = 1.0e-3;

kinematics arm_solver()
{
    const capabilities arm = manipulator::baseline();

    return kinematics::compose(three_link_arm(), arm.fk, arm.dk, arm.ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).value();
}

transform tool_offset()
{
    return rigid_motion::transformation_matrix_from_rotation_position(rigid_motion::rotate_z(0.3), Eigen::Vector3d(0.1, 0.05, 0.0));
}

expected<transform, refusal> reached_under(const rigid_motion::capabilities &, const screw_chain &arm, const joint_vector &at)
{
    return manipulator::baseline().fk.forward_kinematics(arm.home, arm.space_screws, at);
}

expected<joint_vector, refusal> swept_under(const rigid_motion::capabilities &spatial, const kinematics &solver, const transform &start, const joint_vector &at)
{
    const robot_ops arm = manipulator::baseline().robot;

    return manipulator::baseline().motion.task_space_screw(spatial.screw, solver, start, Eigen::Vector3d::UnitZ(), arm.position_from_pose(start), turn, 0.0, at);
}

expected<joint_vector, refusal> displaced_under(const rigid_motion::capabilities &, const kinematics &solver, const transform &start, const joint_vector &at)
{
    return manipulator::baseline().motion.tool_frame_displace(solver, start, Eigen::Vector3d(0.05, 0.0, 0.0), rigid_motion::rotate_z(0.1), at);
}

}

TEST_CASE("a substituted inverse changes the flange pose, which is handed the aggregate it is substituted in", "[seam][routing]")
{
    const rigid_motion::capabilities reference = rigid_motion::baseline();
    const rigid_motion::capabilities perturbed = only_the_inverse();
    const transform offset                     = tool_offset();
    const transform tool_pose                  = rigid_motion::transformation_matrix_from_rotation_position(rigid_motion::rotate_z(-0.7), Eigen::Vector3d(0.4, -0.2, 0.9));

    REQUIRE_FALSE(is_approx_equal(perturbed.frame.inverse(offset), reference.frame.inverse(offset)));

    const robot_ops arm                 = manipulator::baseline().robot;
    const transform under_the_reference = arm.flange_pose_from_tool_pose(reference.frame, tool_pose, offset);
    const transform under_substitution  = arm.flange_pose_from_tool_pose(perturbed.frame, tool_pose, offset);

    REQUIRE_FALSE(is_approx_equal(under_substitution, under_the_reference));
    REQUIRE((under_substitution - under_the_reference).norm() > 1.0);
}

TEST_CASE("forward kinematics answers from the solver library whatever a composition binds for the screw exponential", "[seam][routing]")
{
    const rigid_motion::capabilities reference = rigid_motion::baseline();
    const rigid_motion::capabilities perturbed = only_the_exponential();
    const screw_axis axis                      = about_z(upper_arm);

    REQUIRE_FALSE(perturbed.screw.matrix_exponential_screw(axis, turn).isApprox(reference.screw.matrix_exponential_screw(axis, turn)));

    const screw_chain arm = three_link_arm();
    const joint_vector at = posed_arm();

    const expected<transform, refusal> under_the_reference = reached_under(reference, arm, at);
    const expected<transform, refusal> under_substitution  = reached_under(perturbed, arm, at);

    REQUIRE(under_the_reference.has_value());
    REQUIRE(under_substitution.has_value());
    REQUIRE(*under_substitution == *under_the_reference);
}

TEST_CASE("the screw-swept task-space motion answers under the screw operations the composition hands it", "[seam][routing]")
{
    const rigid_motion::capabilities reference = rigid_motion::baseline();
    const rigid_motion::capabilities perturbed = substituted_everywhere();
    const kinematics solver                    = arm_solver();
    const joint_vector at                      = posed_arm();
    const transform start                      = solver.fk_solve(at).value();

    REQUIRE(every_substituted_slot_differs(reference, perturbed));

    const expected<joint_vector, refusal> under_the_reference = swept_under(reference, solver, start, at);
    const expected<joint_vector, refusal> under_substitution  = swept_under(perturbed, solver, start, at);

    REQUIRE(under_the_reference.has_value());
    REQUIRE(under_substitution.has_value());
    REQUIRE((*under_substitution - *under_the_reference).cwiseAbs().maxCoeff() > apart_by_radians);
}

TEST_CASE("the offset task-space motion answers the same under a composition whose operations it does not reach", "[seam][routing]")
{
    const rigid_motion::capabilities reference = rigid_motion::baseline();
    const rigid_motion::capabilities perturbed = substituted_everywhere();
    const kinematics solver                    = arm_solver();
    const joint_vector at                      = posed_arm();
    const transform start                      = solver.fk_solve(at).value();

    REQUIRE(every_substituted_slot_differs(reference, perturbed));

    SECTION("the offset task-space motion")
    {
        const expected<joint_vector, refusal> under_the_reference = displaced_under(reference, solver, start, at);
        const expected<joint_vector, refusal> under_substitution  = displaced_under(perturbed, solver, start, at);

        REQUIRE(under_the_reference.has_value());
        REQUIRE(under_substitution.has_value());
        REQUIRE(*under_substitution == *under_the_reference);
    }
}
