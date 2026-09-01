#include "configuration_run.h"

#include "praxis/trajectory/baseline/trajectory.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <vector>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::trajectory;

TEST_CASE("a_single_waypoint_is_a_straight_line_from_the_seed")
{
    const configuration seed             = pair_of(0.0, 0.0);
    const std::vector<configuration> via = {pair_of(2.0, 1.0)};
    const auto motion                    = commanded(via, seed);

    CHECK(is_approx_equal(at(*motion, 0.0).position, seed));
    CHECK(is_approx_equal(at(*motion, motion->duration()).position, via.front()));
    CHECK(is_approx_equal(at(*motion, 0.5 * motion->duration()).position, pair_of(1.0, 0.5)));
}

// The slowest degree of freedom sets the pace, and which one that is has to be decided per motion:
// the first target is bound by the second one's lower velocity limit and the second target by the
// first one's longer travel.
TEST_CASE("the_duration_is_the_slowest_degree_of_freedom_s_traversal_at_its_velocity_bound")
{
    const configuration seed                     = pair_of(0.0, 0.0);
    const std::vector<configuration> slow_axis   = {pair_of(1.0, 1.0)};
    const std::vector<configuration> long_travel = {pair_of(3.0, 0.5)};

    CHECK(is_approx_equal(commanded(slow_axis, seed)->duration(), 2.0));
    CHECK(is_approx_equal(commanded(long_travel, seed)->duration(), 3.0));
}

TEST_CASE("a_pair_of_waypoints_runs_between_them_and_not_from_the_seed")
{
    const std::vector<configuration> via = {pair_of(1.0, 1.0), pair_of(2.0, 1.0)};
    const auto motion                    = commanded(via, pair_of(-5.0, -5.0));

    CHECK(is_approx_equal(at(*motion, 0.0).position, via.front()));
    CHECK(is_approx_equal(at(*motion, motion->duration()).position, via.back()));
}

TEST_CASE("the_reported_velocity_is_constant_inside_the_motion_and_zero_at_both_ends")
{
    const std::vector<configuration> via = {pair_of(2.0, 1.0)};
    const auto motion                    = commanded(via, pair_of(0.0, 0.0));

    CHECK(is_approx_equal(at(*motion, 0.25 * motion->duration()).velocity, pair_of(1.0, 0.5)));
    CHECK(is_approx_equal(at(*motion, 0.75 * motion->duration()).velocity, pair_of(1.0, 0.5)));
    CHECK(at(*motion, 0.0).velocity.isZero(default_tolerance));
    CHECK(at(*motion, motion->duration()).velocity.isZero(default_tolerance));
    CHECK(at(*motion, 0.5 * motion->duration()).acceleration.isZero(default_tolerance));
}

TEST_CASE("sampling_a_configuration_space_motion_is_a_pure_function_of_the_time_it_is_given")
{
    const std::vector<configuration> via = {pair_of(2.0, 1.0)};
    const auto motion                    = commanded(via, pair_of(0.0, 0.0));

    CHECK(is_approx_equal(at(*motion, 1.3).position, at(*motion, 1.3).position));
    CHECK(is_approx_equal(at(*motion, 0.4).position, pair_of(0.4, 0.2)));
    CHECK(is_approx_equal(at(*motion, -1.0).position, pair_of(0.0, 0.0)));
    CHECK(is_approx_equal(at(*motion, 50.0).position, via.front()));
}

TEST_CASE("a_run_holding_no_waypoint_at_all_is_refused_rather_than_discarded")
{
    const configuration seed = pair_of(0.3, -0.2);

    const auto empty = joint_space_waypoints({}, seed, bounds());

    REQUIRE(!empty);
    CHECK(empty.error() == refusal::unsupported_input);
}

TEST_CASE("two_configurations_of_different_size_are_refused_rather_than_joined")
{
    const configuration seed             = configuration::Zero(3);
    const std::vector<configuration> via = {pair_of(1.0, 1.0)};

    const auto refused = joint_space_waypoints(via, seed, bounds());

    REQUIRE(!refused);
    CHECK(refused.error() == refusal::unsupported_input);
}

TEST_CASE("a_single_waypoint_holds_a_constant_speed_and_no_acceleration_from_the_seed")
{
    const std::vector<configuration> via = {pair_of(2.0, 1.0)};
    const auto motion                    = commanded(via, pair_of(0.0, 0.0));
    const trajectory_sample sampled      = at(*motion, 0.4 * motion->duration());

    CHECK(is_approx_equal(motion->duration(), 2.0));
    CHECK(is_approx_equal(sampled.position, pair_of(0.8, 0.4)));
    CHECK(is_approx_equal(sampled.velocity, pair_of(1.0, 0.5)));
    CHECK(sampled.acceleration.isZero(default_tolerance));
}

TEST_CASE("a_pair_of_waypoints_holds_a_constant_speed_and_no_acceleration_between_them")
{
    const std::vector<configuration> via = {pair_of(1.0, 1.0), pair_of(2.0, 1.0)};
    const auto motion                    = commanded(via, pair_of(-5.0, -5.0));
    const trajectory_sample sampled      = at(*motion, 0.4 * motion->duration());

    CHECK(is_approx_equal(motion->duration(), 1.0));
    CHECK(is_approx_equal(sampled.position, pair_of(1.4, 1.0)));
    CHECK(is_approx_equal(sampled.velocity, pair_of(1.0, 0.0)));
    CHECK(sampled.acceleration.isZero(default_tolerance));
}
