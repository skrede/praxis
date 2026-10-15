#include "fixtures.h"

#include "book_trapezoid_options.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

// Spelled as printed and checking nothing, so a span below the natural duration takes the square
// root of a negative number.
constexpr std::size_t printed_unchecked = 0u;

std::size_t literal_asks = 0;

expected<trajectory::scaling_sample, refusal> literal_velocity_acceleration(double t, double velocity, double acceleration)
{
    ++literal_asks;

    return book_velocity_acceleration_option<printed_unchecked>(t, velocity, acceleration);
}

expected<trajectory::scaling_sample, refusal> literal_acceleration_duration(double t, double duration, double acceleration)
{
    ++literal_asks;

    return book_acceleration_duration_option<printed_unchecked>(t, duration, acceleration);
}

// The previewed target lies 0.8 rad away in both joints, which the arm's limits give a quintic span of
// 1.5 s.
std::shared_ptr<const preview_run> quintic_preview_under(path_parameter_bounds held)
{
    trajectory::time_scaling_ops scaling           = trajectory::baseline().time_scaling;
    scaling.trapezoidal_from_velocity_acceleration = &literal_velocity_acceleration;
    scaling.trapezoidal_from_acceleration_duration = &literal_acceleration_duration;
    const manipulator::capabilities arm            = manipulator::baseline();
    const rigid_motion::capabilities rigid         = rigid_motion::baseline();
    const trajectory::capabilities shapes          = trajectory::baseline();

    scene_robot robot = two_joint_arm(arm.robot);
    robot.set_joint_positions(configuration(0.0, 0.0));
    robot_controller control(robot, arm.motion, shapes.path, arm.trajectory, scaling, shapes.trajectory, rigid.screw, rigid.frame);
    control.set_trapezoid_bounds(held);
    literal_asks = 0;

    control.preview_trajectory(configuration(0.8, 0.8));

    return control.preview();
}

bool finite(const std::vector<trajectory::scaling_sample> &curve)
{
    for(const trajectory::scaling_sample &one : curve)
        if(!std::isfinite(one.s) || !std::isfinite(one.ds) || !std::isfinite(one.dds))
            return false;

    return true;
}

}

// Natural duration 2.5 s for these bounds, a whole second beyond the span.
TEST_CASE("a trapezoid curve over a span shorter than its natural duration is left empty and asks no trapezoid", "[manipulator][trajectory]")
{
    const std::shared_ptr<const preview_run> run = quintic_preview_under(path_parameter_bounds{0.5, 1.0});

    REQUIRE(run != nullptr);
    CHECK(run->scaling[2].empty());
    CHECK(literal_asks == 0u);
    CHECK(run->scaling[0].size() == run->samples.size());
    CHECK(run->scaling[1].size() == run->samples.size());
    CHECK(finite(run->scaling[0]));
    CHECK(finite(run->scaling[1]));
    CHECK(finite(run->parameter));
}

// Natural duration 0.75 s for these bounds, half the span.
TEST_CASE("a trapezoid curve over a span longer than its natural duration is drawn whole and ends at the path's end", "[manipulator][trajectory]")
{
    const std::shared_ptr<const preview_run> run = quintic_preview_under(path_parameter_bounds{2.0, 8.0});

    REQUIRE(run != nullptr);
    REQUIRE(run->scaling[2].size() == run->samples.size());
    CHECK(literal_asks >= run->samples.size());
    CHECK(finite(run->scaling[2]));
    CHECK(std::abs(run->scaling[2].back().s - 1.0) <= 1.0e-12);
}
