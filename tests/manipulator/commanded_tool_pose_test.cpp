#include "commanded_tool_pose.h"

#include "praxis/manipulator/arm_snapshot.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <span>
#include <limits>
#include <memory>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

// How far apart in any one joint two configurations must stand before they are different answers,
// far above the step the solve stops within.
constexpr double apart_by_radians = 1.0e-3;

constexpr double thirty_degrees = 30.0 * radians_per_degree;

constexpr std::size_t waypoints_in_a_run = 3;

constexpr double turn_between_waypoints = 0.1;

constexpr double recorded_exactly = 1.0e-12;

Eigen::Vector3d along_the_tool_x()
{
    return Eigen::Vector3d(jog_tick(), 0.0, 0.0);
}

rotation thirty_degrees_about_z()
{
    return rigid_motion::rotate_z(thirty_degrees);
}

void jog(commanded_arm &placed, const Eigen::Vector3d &offset, const rotation &turned)
{
    placed.control().preview_tool_frame_jog(placed.tool_pose(), offset, turned);
}

double travel_of(commanded_arm &placed, const Eigen::Vector3d &offset, const rotation &turned)
{
    return tool_travel(placed, [&placed, &offset, &turned] { jog(placed, offset, turned); });
}

// As rigid as the reference conversion and landing elsewhere: the inverse offset composes on the left.
transform flange_pose_with_the_offset_on_the_left(const rigid_motion::frame_ops &frames, const transform &tool_pose, const transform &tool_offset)
{
    return frames.inverse(tool_offset) * tool_pose;
}

robot_ops converting_on_the_left()
{
    robot_ops injected                  = manipulator::baseline().robot;
    injected.flange_pose_from_tool_pose = &flange_pose_with_the_offset_on_the_left;

    return injected;
}

transform nudged(const transform &pose)
{
    return pose * rigid_motion::transformation_matrix_from_position(Eigen::Vector3d(0.03, 0.02, 0.0));
}

double apart(const transform &reached, const transform &wanted)
{
    return (reached - wanted).cwiseAbs().maxCoeff();
}

transform turning_about(const Eigen::Vector3d &through)
{
    const expected<screw_axis, refusal> axis = rigid_motion::screw_axis_from_point_direction_pitch(through, Eigen::Vector3d::UnitZ(), 0.0);

    return rigid_motion::matrix_exponential_screw(*axis, thirty_degrees);
}

struct commanded_outcome
{
    bool composed;
    double strays;
};

commanded_outcome played_to(commanded_arm &placed, const transform &target)
{
    const bool composed = placed.control().executing();
    if(!played_out(placed.control()))
        return commanded_outcome{composed, std::numeric_limits<double>::infinity()};

    return commanded_outcome{composed, apart(placed.tool_pose(), target)};
}

double pose_preview_strays(const transform &tool_offset)
{
    commanded_arm placed(tool_offset);
    const transform target = nudged(placed.tool_pose());

    placed.control().preview_task_space_pose(target);

    return apart(placed.tool_pose(), target);
}

commanded_outcome point_to_point_outcome(const transform &tool_offset)
{
    commanded_arm placed(tool_offset);
    const transform target = nudged(placed.tool_pose());

    placed.control().task_space_ptp(target);

    return played_to(placed, target);
}

commanded_outcome linear_outcome(const transform &tool_offset)
{
    commanded_arm placed(tool_offset);
    const transform target = nudged(placed.tool_pose());

    placed.control().task_space_lin(target);

    return played_to(placed, target);
}

double screw_preview_strays(const transform &tool_offset)
{
    commanded_arm placed(tool_offset);
    const Eigen::Vector3d through = placed.tool_position();
    const transform start         = placed.tool_pose();
    const transform target        = turning_about(through) * start;

    placed.control().preview_task_space_screw(start, Eigen::Vector3d::UnitZ(), through, thirty_degrees, 0.0);

    return apart(placed.tool_pose(), target);
}

commanded_outcome screw_outcome(const transform &tool_offset)
{
    commanded_arm placed(tool_offset);
    const Eigen::Vector3d through = placed.tool_position();
    const transform target        = turning_about(through) * placed.tool_pose();

    placed.control().task_space_screw(Eigen::Vector3d::UnitZ(), through, thirty_degrees, 0.0);

    return played_to(placed, target);
}

transform stepped(const transform &pose)
{
    return pose * rigid_motion::transformation_matrix_from_rotation_position(rigid_motion::rotate_z(turn_between_waypoints), Eigen::Vector3d(0.03, 0.02, 0.0));
}

std::vector<transform> run_from(const transform &pose)
{
    std::vector<transform> through;
    transform at = pose;
    for(std::size_t step = 0; step < waypoints_in_a_run; ++step)
    {
        at = stepped(at);
        through.push_back(at);
    }

    return through;
}

commanded_outcome waypoint_run_outcome(const transform &tool_offset)
{
    commanded_arm placed(tool_offset);
    const std::vector<transform> through = run_from(placed.tool_pose());

    placed.control().task_space_trajectory(std::span<const transform>(through));

    return played_to(placed, through.back());
}

double previewed_run_strays(const transform &tool_offset)
{
    commanded_arm placed(tool_offset);
    const std::vector<transform> through = run_from(placed.tool_pose());

    placed.control().preview_trajectory(std::span<const transform>(through));

    const std::shared_ptr<const preview_run> drawn = placed.control().preview();
    if(!drawn || drawn->samples.empty())
        return std::numeric_limits<double>::infinity();

    placed.driven().set_joint_positions(drawn->samples.back().motion.position);

    return apart(placed.tool_pose(), through.back());
}

double previewed_path_strays(const transform &tool_offset)
{
    commanded_arm placed(tool_offset);
    const transform target = nudged(placed.tool_pose());

    placed.control().preview_trajectory(target);

    const std::shared_ptr<const preview_run> drawn = placed.control().preview();
    if(!drawn || drawn->samples.empty())
        return std::numeric_limits<double>::infinity();

    placed.driven().set_joint_positions(drawn->samples.back().motion.position);

    return apart(placed.tool_pose(), target);
}

joint_vector answered_configuration()
{
    return arm_configuration(0.5, -1.7, 1.0);
}

transform &pose_the_closed_form_was_asked_at()
{
    static transform asked = transform::Identity();

    return asked;
}

expected<void, refusal> recorded_closed_form(const rigid_motion::screw_ops &, const forward_kinematics_ops &, const screw_chain &, const transform &desired, ik_result &answer)
{
    pose_the_closed_form_was_asked_at() = desired;
    answer.solutions.push_back(answered_configuration());

    return {};
}

inverse_kinematics_ops recording_the_closed_form()
{
    inverse_kinematics_ops injected      = manipulator::baseline().ik;
    injected.analytic_inverse_kinematics = &recorded_closed_form;

    return injected;
}

commanded_outcome seeded_solve_outcome(const transform &tool_offset)
{
    commanded_arm placed(tool_offset);
    const transform target = nudged(placed.tool_pose());
    const std::vector<joint_vector> from{placed.driven().joint_positions()};

    placed.control().solve_from_seeds(target, std::span<const joint_vector>(from));

    return played_to(placed, target);
}

double closed_form_asked_at_strays(const transform &tool_offset)
{
    commanded_arm placed(tool_offset, manipulator::baseline().robot, recording_the_closed_form());
    const transform target   = nudged(placed.tool_pose());
    const transform reaching = placed.driven().flange_pose_from_tool_pose(target);

    pose_the_closed_form_was_asked_at() = transform::Identity();
    placed.control().solve_in_closed_form(target);

    return apart(pose_the_closed_form_was_asked_at(), reaching);
}

}

TEST_CASE("a tick along a bent tool's own x moves the tool centre point by the tick", "[manipulator]")
{
    commanded_arm placed(bent_tool());

    CHECK(travel_of(placed, along_the_tool_x(), rotation::Identity()) == Catch::Approx(jog_tick()).margin(solved_tolerance));
}

TEST_CASE("a tick along a straight tool's own x moves the tool centre point by the tick", "[manipulator]")
{
    commanded_arm placed(straight_tool());

    CHECK(travel_of(placed, along_the_tool_x(), rotation::Identity()) == Catch::Approx(jog_tick()).margin(solved_tolerance));
}

TEST_CASE("a tick along the tool x of an arm wearing no tool moves the tool centre point by the tick", "[manipulator]")
{
    commanded_arm placed(no_tool());

    CHECK(travel_of(placed, along_the_tool_x(), rotation::Identity()) == Catch::Approx(jog_tick()).margin(solved_tolerance));
}

TEST_CASE("a jog of no offset and no turn leaves a tooled arm's configuration where it stood", "[manipulator]")
{
    commanded_arm placed(bent_tool());
    const joint_vector stood = placed.driven().joint_positions();

    jog(placed, Eigen::Vector3d::Zero(), rotation::Identity());

    CHECK((placed.driven().joint_positions() - stood).cwiseAbs().maxCoeff() < solved_tolerance);
}

TEST_CASE("turning a bent tool's jog thirty degrees about z leaves the tool centre point where it stood", "[manipulator]")
{
    commanded_arm placed(bent_tool());

    CHECK(travel_of(placed, Eigen::Vector3d::Zero(), thirty_degrees_about_z()) < solved_tolerance);
}

TEST_CASE("a jog lands at the configuration the bound frame conversion reaches and not another", "[manipulator]")
{
    commanded_arm reference(bent_tool());
    commanded_arm substituted(bent_tool(), converting_on_the_left());

    jog(reference, along_the_tool_x(), rotation::Identity());
    jog(substituted, along_the_tool_x(), rotation::Identity());

    CHECK((substituted.driven().joint_positions() - reference.driven().joint_positions()).cwiseAbs().maxCoeff() > apart_by_radians);
}

TEST_CASE("a previewed task-space pose stands a bent tool at the pose it was given", "[manipulator]")
{
    CHECK(pose_preview_strays(bent_tool()) < solved_tolerance);
}

TEST_CASE("a previewed task-space pose stands an arm wearing no tool at the pose it was given", "[manipulator]")
{
    CHECK(pose_preview_strays(no_tool()) < solved_tolerance);
}

TEST_CASE("a point-to-point command ends with a bent tool at the pose it was given", "[manipulator]")
{
    const commanded_outcome outcome = point_to_point_outcome(bent_tool());

    CHECK(outcome.composed);
    CHECK(outcome.strays < solved_tolerance);
}

TEST_CASE("a point-to-point command ends with an arm wearing no tool at the pose it was given", "[manipulator]")
{
    const commanded_outcome outcome = point_to_point_outcome(no_tool());

    CHECK(outcome.composed);
    CHECK(outcome.strays < solved_tolerance);
}

TEST_CASE("a linear command is composed on a bent tool and ends with it at the pose it was given", "[manipulator]")
{
    const commanded_outcome outcome = linear_outcome(bent_tool());

    CHECK(outcome.composed);
    CHECK(outcome.strays < solved_tolerance);
}

TEST_CASE("a linear command is composed on an arm wearing no tool and ends with it at the pose it was given", "[manipulator]")
{
    const commanded_outcome outcome = linear_outcome(no_tool());

    CHECK(outcome.composed);
    CHECK(outcome.strays < solved_tolerance);
}

TEST_CASE("a previewed screw turns a bent tool about the axis it was given", "[manipulator]")
{
    CHECK(screw_preview_strays(bent_tool()) < solved_tolerance);
}

TEST_CASE("a previewed screw turns an arm wearing no tool about the axis it was given", "[manipulator]")
{
    CHECK(screw_preview_strays(no_tool()) < solved_tolerance);
}

TEST_CASE("a screw command turns a bent tool about an axis through its own origin and leaves that origin where it stood", "[manipulator]")
{
    const commanded_outcome outcome = screw_outcome(bent_tool());

    CHECK(outcome.composed);
    CHECK(outcome.strays < solved_tolerance);
}

TEST_CASE("a screw command turns an arm wearing no tool about an axis through its own origin and leaves that origin where it stood", "[manipulator]")
{
    const commanded_outcome outcome = screw_outcome(no_tool());

    CHECK(outcome.composed);
    CHECK(outcome.strays < solved_tolerance);
}

TEST_CASE("the last sample of a previewed task-space motion stands a bent tool at the pose it was given", "[manipulator]")
{
    CHECK(previewed_path_strays(bent_tool()) < solved_tolerance);
}

TEST_CASE("the last sample of a previewed task-space motion stands an arm wearing no tool at the pose it was given", "[manipulator]")
{
    CHECK(previewed_path_strays(no_tool()) < solved_tolerance);
}

TEST_CASE("a commanded run of poses ends with a bent tool at the last of them", "[manipulator]")
{
    const commanded_outcome outcome = waypoint_run_outcome(bent_tool());

    CHECK(outcome.composed);
    CHECK(outcome.strays < solved_tolerance);
}

TEST_CASE("a commanded run of poses ends with an arm wearing no tool at the last of them", "[manipulator]")
{
    const commanded_outcome outcome = waypoint_run_outcome(no_tool());

    CHECK(outcome.composed);
    CHECK(outcome.strays < solved_tolerance);
}

TEST_CASE("the last sample of a previewed run of poses stands a bent tool at the last of them", "[manipulator]")
{
    CHECK(previewed_run_strays(bent_tool()) < solved_tolerance);
}

TEST_CASE("the last sample of a previewed run of poses stands an arm wearing no tool at the last of them", "[manipulator]")
{
    CHECK(previewed_run_strays(no_tool()) < solved_tolerance);
}

TEST_CASE("a solve from a set of starts ends with a bent tool at the pose it was asked at", "[manipulator]")
{
    const commanded_outcome outcome = seeded_solve_outcome(bent_tool());

    CHECK(outcome.composed);
    CHECK(outcome.strays < solved_tolerance);
}

TEST_CASE("a solve from a set of starts ends with an arm wearing no tool at the pose it was asked at", "[manipulator]")
{
    const commanded_outcome outcome = seeded_solve_outcome(no_tool());

    CHECK(outcome.composed);
    CHECK(outcome.strays < solved_tolerance);
}

TEST_CASE("a closed-form solve is asked at the flange pose a bent tool's commanded pose converts to", "[manipulator]")
{
    CHECK(closed_form_asked_at_strays(bent_tool()) < recorded_exactly);
}

TEST_CASE("a closed-form solve is asked at the flange pose the commanded pose of an arm wearing no tool converts to", "[manipulator]")
{
    CHECK(closed_form_asked_at_strays(no_tool()) < recorded_exactly);
}
