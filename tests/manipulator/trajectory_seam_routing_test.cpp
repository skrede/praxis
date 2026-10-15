#include "seam_stage.h"
#include "substituted_trajectory.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <memory>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

TEST_CASE("a trapezoidal run is timed and sampled by the velocity and acceleration trapezoid the composition binds", "[seam][routing]")
{
    const composed_arm arm = arm_under(only_the_velocity_acceleration_trapezoid());
    const std::array<joint_vector, 1> target{configuration(0.8, 0.8)};
    arm.control->set_time_scaling(time_scaling_choice::trapezoidal);
    reset_counts();

    arm.control->run_in_turn(target);

    REQUIRE(played_out(*arm.control));
    CHECK(is_approx_equal(arm.robot->joint_positions(), target.front(), 1.0e-9));
    CHECK(velocity_acceleration_asks() > 0u);
}

// The motion and the curve beside it are both sampled at every previewed instant, so a count of at
// least twice the samples is a count the curve contributed to.
TEST_CASE("a trapezoidal motion's own preview curve is drawn by the velocity and acceleration trapezoid", "[seam][routing]")
{
    const composed_arm arm = arm_under(only_the_velocity_acceleration_trapezoid());
    arm.control->set_time_scaling(time_scaling_choice::trapezoidal);
    reset_counts();

    arm.control->preview_trajectory(configuration(0.8, 0.8));

    const std::shared_ptr<const preview_run> run = arm.control->preview();
    REQUIRE(run != nullptr);
    CHECK(run->scaling[2].size() == run->samples.size());
    CHECK(velocity_acceleration_asks() >= 2u * run->samples.size());
}

TEST_CASE("a trapezoidal motion and its own preview curve never ask the acceleration and duration trapezoid", "[seam][routing]")
{
    const composed_arm arm = arm_under(only_the_acceleration_duration_trapezoid());
    arm.control->set_time_scaling(time_scaling_choice::trapezoidal);
    reset_counts();

    arm.control->preview_trajectory(configuration(0.8, 0.8));

    const std::shared_ptr<const preview_run> run = arm.control->preview();
    REQUIRE(run != nullptr);
    CHECK(run->scaling[2].size() == run->samples.size());
    CHECK(acceleration_duration_asks() == 0u);
}

// A reach this short has a natural duration far below the floor every motion keeps.
TEST_CASE("a floored trapezoidal run is timed and sampled by the acceleration and duration trapezoid the composition binds", "[seam][routing]")
{
    const composed_arm arm = arm_under(only_the_acceleration_duration_trapezoid());
    const std::array<joint_vector, 1> target{configuration(1.0e-4, 1.0e-4)};
    arm.control->set_time_scaling(time_scaling_choice::trapezoidal);
    reset_counts();

    arm.control->run_in_turn(target);

    REQUIRE(played_out(*arm.control));
    CHECK(is_approx_equal(arm.robot->joint_positions(), target.front(), 1.0e-9));
    CHECK(acceleration_duration_asks() > 0u);
}

TEST_CASE("the trapezoid curve over a quintic motion's span is drawn by the acceleration and duration trapezoid", "[seam][routing]")
{
    const composed_arm arm = arm_under(only_the_acceleration_duration_trapezoid());
    reset_counts();

    arm.control->preview_trajectory(configuration(0.8, 0.8));

    const std::shared_ptr<const preview_run> run = arm.control->preview();
    REQUIRE(run != nullptr);
    CHECK(arm.control->time_scaling() == time_scaling_choice::quintic);
    CHECK(run->scaling[2].size() == run->samples.size());
    CHECK(acceleration_duration_asks() >= run->samples.size());
}
