#include "commanded_tool_pose.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/baseline/frame.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

// How far apart in any one joint two configurations must stand before they are different answers,
// far above the step the solve stops within.
constexpr double apart_by_radians = 1.0e-3;

Eigen::Vector3d along_the_tool_x()
{
    return Eigen::Vector3d(jog_tick(), 0.0, 0.0);
}

rotation thirty_degrees_about_z()
{
    return rigid_motion::rotate_z(30.0 * radians_per_degree);
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
