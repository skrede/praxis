#include "joint_turns.h"

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/motion.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/screw_chain.h"

#include "captured_log.h"

#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/optimization/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <numbers>
#include <optional>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

constexpr double turn  = 2.0 * std::numbers::pi;
constexpr double exact = 1.0e-12;

// A revolute joint about z through the origin, turning within [-2, 6], and a prismatic joint along
// x, sliding within [-1, 1].
screw_chain turning_and_sliding()
{
    screw_axis about_z = screw_axis::Zero();
    about_z[2]         = 1.0;
    screw_axis along_x = screw_axis::Zero();
    along_x[3]         = 1.0;

    joint_limits bounds{};
    bounds.velocity       = joint_vector::Constant(2, 1.0);
    bounds.acceleration   = joint_vector::Constant(2, 4.0);
    bounds.lower_position = joint_vector{{-2.0, -1.0}};
    bounds.upper_position = joint_vector{{6.0, 1.0}};

    return screw_chain(transform::Identity(), {about_z, along_x}, bounds);
}

kinematics turning_solver()
{
    const capabilities arm = manipulator::baseline();

    return kinematics::compose(turning_and_sliding(), arm.fk, arm.dk, arm.ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame, optimization::baseline().solver).value();
}

std::vector<joint_vector> answers;
std::size_t answered = 0;

expected<joint_vector, refusal> answering_in_turn(const kinematics &, const transform &, const joint_vector &)
{
    return answers.at(answered++);
}

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> resolved_through(std::vector<joint_vector> told, const joint_vector &j0)
{
    answers  = std::move(told);
    answered = 0;

    const kinematics solver = turning_solver();
    const std::vector<transform> poses(answers.size(), transform::Identity());

    return manipulator::baseline().trajectory.task_space_waypoints(trajectory::baseline().trajectory, motion_ops{.task_space_pose = &answering_in_turn}, solver, poses, j0,
                                                                   solver.space_chain().limits);
}

joint_vector joints(double revolute, double prismatic)
{
    return joint_vector{{revolute, prismatic}};
}

}

TEST_CASE("a revolute joint whole turns away is named at its equivalent nearest the configuration it is held against", "[manipulator][turns]")
{
    const std::optional<joint_vector> turned = turned_within_limits(turning_and_sliding(), joints(0.5 + 3.0 * turn, 0.25), joints(0.0, 0.0));

    REQUIRE(turned.has_value());
    CHECK(std::abs((*turned)[0] - 0.5) < exact);
    CHECK((*turned)[1] == 0.25);
}

TEST_CASE("the equivalent nearest the held configuration is passed over for the nearest one inside the limits", "[manipulator][turns]")
{
    const std::optional<joint_vector> turned = turned_within_limits(turning_and_sliding(), joints(0.5, 0.0), joints(6.0, 0.0));

    REQUIRE(turned.has_value());
    CHECK((*turned)[0] == 0.5);
}

TEST_CASE("a revolute joint landing on a bound a whole turn away is named on that bound", "[manipulator][turns]")
{
    const std::optional<joint_vector> turned = turned_within_limits(turning_and_sliding(), joints(-2.0 - turn, 0.0), joints(-2.0, 0.0));

    REQUIRE(turned.has_value());
    CHECK((*turned)[0] == -2.0);
}

TEST_CASE("a prismatic joint outside its limits has no equivalent inside them", "[manipulator][turns]")
{
    CHECK_FALSE(turned_within_limits(turning_and_sliding(), joints(0.0, 1.25), joints(0.0, 0.0)).has_value());
}

TEST_CASE("a configuration that is not one entry per joint has no equivalent", "[manipulator][turns]")
{
    CHECK_FALSE(turned_within_limits(turning_and_sliding(), joint_vector::Zero(3), joints(0.0, 0.0)).has_value());
    CHECK_FALSE(turned_within_limits(turning_and_sliding(), joints(0.0, 0.0), joint_vector::Zero(1)).has_value());
}

TEST_CASE("a waypoint resolved whole turns away is moved to from its equivalent nearest the configuration before it", "[manipulator][turns]")
{
    const auto motion = resolved_through({joints(0.3 + 4.0 * turn, 0.1)}, joints(0.0, 0.0));

    REQUIRE(motion.has_value());
    const expected<trajectory::trajectory_sample, refusal> end = (*motion)->sample((*motion)->duration());
    REQUIRE(end.has_value());
    CHECK(std::abs(end->position[0] - 0.3) < 1.0e-9);
    CHECK(std::abs(end->position[1] - 0.1) < 1.0e-9);
}

TEST_CASE("a waypoint resolved only outside the joint limits refuses the run", "[manipulator][turns]")
{
    tests::captured_log captured;

    const auto motion = resolved_through({joints(0.1, 0.0), joints(0.2, 1.25)}, joints(0.0, 0.0));

    REQUIRE_FALSE(motion.has_value());
    CHECK(motion.error() == refusal::no_solution);
    CHECK_THAT(captured.text(), Catch::Matchers::ContainsSubstring("waypoint 2 of 2 resolved to a configuration no whole turn of its joints brings inside the joint limits"));
}

TEST_CASE("the largest turn of one joint between consecutive waypoints is reported once it passes the reported turn", "[manipulator][turns]")
{
    tests::captured_log captured;

    const auto motion = resolved_through({joints(0.1, 0.0), joints(0.9, 0.0), joints(1.0, 0.0)}, joints(0.0, 0.0));

    REQUIRE(motion.has_value());
    CHECK_THAT(captured.text(), Catch::Matchers::ContainsSubstring("joint 1 turns 45.8 degrees between waypoint 1 and waypoint 2 of 3"));
}

TEST_CASE("a turn from the starting configuration is reported against it", "[manipulator][turns]")
{
    tests::captured_log captured;

    const auto motion = resolved_through({joints(0.0, 0.0), joints(0.05, 0.0)}, joints(1.0, 0.0));

    REQUIRE(motion.has_value());
    CHECK_THAT(captured.text(), Catch::Matchers::ContainsSubstring("joint 1 turns 57.3 degrees between the starting configuration and waypoint 1 of 2"));
}

TEST_CASE("waypoints no joint turns far between are resolved without a report", "[manipulator][turns]")
{
    tests::captured_log captured;

    const auto motion = resolved_through({joints(0.1, 0.1), joints(0.2, 0.2), joints(0.3, 0.3)}, joints(0.0, 0.0));

    REQUIRE(motion.has_value());
    CHECK(captured.text().empty());
}
