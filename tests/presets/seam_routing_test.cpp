#include "three_link_arm.h"
#include "substituted_rigid_motion.h"

#include "praxis/manipulator/robot.h"
#include "praxis/manipulator/motion.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/motion_commands.h"

#include "praxis/trajectory/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <memory>
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

expected<transform, refusal> reached_under(const rigid_motion::capabilities &spatial, const screw_chain &arm, const joint_vector &at)
{
    return manipulator::baseline().fk.forward_kinematics(spatial.screw, arm.home, arm.space_screws, at);
}

expected<joint_vector, refusal> swept_under(const rigid_motion::capabilities &spatial, const kinematics &solver, const transform &start, const joint_vector &at)
{
    const robot_ops arm = manipulator::baseline().robot;

    return manipulator::baseline().motion.task_space_screw(spatial.screw, solver, start, Eigen::Vector3d::UnitZ(), arm.position_from_pose(start), turn, 0.0, at);
}

// The arm's configuration halfway through a task-space motion from the posed arm to a second planar
// pose, the motion composed under the rigid-motion operations given.
expected<joint_vector, refusal> midway_along(const rigid_motion::capabilities &spatial, const kinematics &solver, task_space_path shape)
{
    const joint_vector at = posed_arm();
    const prepared_time_scaling scaled(trajectory::baseline().time_scaling, time_scaling_choice::quintic);
    const transform from = solver.fk_solve(at).value();
    const transform to   = solver.fk_solve(arm_configuration(0.9, -0.6, -0.3)).value();

    const expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> motion =
            task_space_motion(manipulator::baseline().motion, spatial.screw, spatial.frame, scaled, solver, three_link_arm().limits, at, from, to, shape);
    if(!motion)
        return unexpected(motion.error());

    const expected<trajectory::trajectory_sample, refusal> midway = (*motion)->sample(0.5 * (*motion)->duration());
    if(!midway)
        return unexpected(midway.error());

    return midway->position;
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

TEST_CASE("forward kinematics answers from its own screw exponential whatever a composition binds for it", "[seam][routing]")
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

TEST_CASE("a straight-line task-space motion answers under the screw and frame operations the composition hands it", "[seam][routing]")
{
    const kinematics solver                                   = arm_solver();
    const task_space_path straight                            = trajectory::baseline().path.decoupled;
    const expected<joint_vector, refusal> under_the_reference = midway_along(rigid_motion::baseline(), solver, straight);
    const expected<joint_vector, refusal> under_substitution  = midway_along(substituted_along_a_path(), solver, straight);

    REQUIRE(every_path_substitution_differs(rigid_motion::baseline(), substituted_along_a_path()));
    REQUIRE(under_the_reference.has_value());
    REQUIRE(under_substitution.has_value());
    REQUIRE((*under_substitution - *under_the_reference).cwiseAbs().maxCoeff() > apart_by_radians);

    // A rotation read as twice itself is no rotation, so the path refuses the start pose it is read from.
    REQUIRE_FALSE(midway_along(substituted_everywhere(), solver, straight).has_value());
}

TEST_CASE("a screw-path task-space motion answers under the screw and frame operations the composition hands it", "[seam][routing]")
{
    const kinematics solver                                   = arm_solver();
    const task_space_path screwed                             = trajectory::baseline().path.screw;
    const expected<joint_vector, refusal> under_the_reference = midway_along(rigid_motion::baseline(), solver, screwed);
    const expected<joint_vector, refusal> under_substitution  = midway_along(substituted_along_a_path(), solver, screwed);

    REQUIRE(every_path_substitution_differs(rigid_motion::baseline(), substituted_along_a_path()));
    REQUIRE(under_the_reference.has_value());
    REQUIRE(under_substitution.has_value());
    REQUIRE((*under_substitution - *under_the_reference).cwiseAbs().maxCoeff() > apart_by_radians);

    // A rotation read as twice itself is no rotation, so the path refuses the start pose it is read from.
    REQUIRE_FALSE(midway_along(substituted_everywhere(), solver, screwed).has_value());
}
