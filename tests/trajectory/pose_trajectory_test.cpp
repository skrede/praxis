#include "praxis/trajectory/pose_trajectory.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <memory>
#include <cstddef>
#include <utility>

using namespace praxis;
using namespace praxis::trajectory;

namespace {

transform seed_pose()
{
    transform tf = transform::Identity();
    tf(0, 3)     = 0.25;
    tf(1, 3)     = -0.5;
    tf(2, 3)     = 1.75;
    return tf;
}

std::unique_ptr<pose_trajectory_generator> inert_generator()
{
    pose_trajectory_ops ops{};
    auto motion = ops.decoupled_pose_waypoints({}, seed_pose(), 1.0, 1.0);
    REQUIRE(motion);

    return std::move(*motion);
}

}

TEST_CASE("the_pose_waypoint_slot_defaults_to_the_inert_factory")
{
    pose_trajectory_ops ops{};

    REQUIRE(ops.decoupled_pose_waypoints == &inert::decoupled_pose_waypoints);
    REQUIRE(ops.screw_pose_waypoints == &inert::screw_pose_waypoints);
    REQUIRE(inert_generator() != nullptr);
}

TEST_CASE("the_inert_pose_generator_reports_a_duration_of_zero")
{
    REQUIRE(is_approx_equal(inert_generator()->duration(), 0.0));
}

// An unbound slot holds no plan, so it answers no pose at all: reporting the seed would read as a
// motion that ran and arrived.
TEST_CASE("the_inert_pose_generator_refuses_to_sample_rather_than_reporting_the_seed")
{
    const std::unique_ptr<pose_trajectory_generator> motion = inert_generator();

    for(double t : std::array<double, 3>{0.0, 0.5, 12.0})
    {
        const expected<pose_sample, refusal> sampled = motion->sample(t);

        REQUIRE(!sampled);
        REQUIRE(sampled.error() == refusal::not_implemented);
    }
}

TEST_CASE("sampling_by_absolute_time_does_not_depend_on_the_order_the_times_arrive_in")
{
    const std::unique_ptr<pose_trajectory_generator> motion = inert_generator();
    const std::array<double, 4> times{0.0, 0.25, 0.75, 2.0};

    for(double t : times)
        REQUIRE(!motion->sample(t));

    for(std::size_t i = times.size(); i > 0; --i)
        REQUIRE(!motion->sample(times[i - 1]));
}
