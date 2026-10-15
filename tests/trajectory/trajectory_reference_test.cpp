#include "configuration_run.h"

#include "praxis/trajectory/baseline/trajectory.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <vector>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::trajectory;

TEST_CASE("a_single_position_is_reached_from_the_seed_through_the_halfway_configuration")
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
TEST_CASE("the_duration_is_one_and_a_half_times_the_slowest_degree_of_freedom_s_traversal_at_its_velocity_bound")
{
    const configuration seed                     = pair_of(0.0, 0.0);
    const std::vector<configuration> slow_axis   = {pair_of(1.0, 1.0)};
    const std::vector<configuration> long_travel = {pair_of(3.0, 0.5)};

    CHECK(is_approx_equal(commanded(slow_axis, seed)->duration(), 3.0));
    CHECK(is_approx_equal(commanded(long_travel, seed)->duration(), 4.5));
}

TEST_CASE("a_pair_of_positions_runs_between_them_and_not_from_the_seed")
{
    const std::vector<configuration> via = {pair_of(1.0, 1.0), pair_of(2.0, 1.0)};
    const auto motion                    = commanded(via, pair_of(-5.0, -5.0));

    CHECK(is_approx_equal(at(*motion, 0.0).position, via.front()));
    CHECK(is_approx_equal(at(*motion, motion->duration()).position, via.back()));
}

TEST_CASE("the_reported_velocity_is_zero_at_both_ends_and_peaks_halfway_at_the_bound")
{
    const std::vector<configuration> via = {pair_of(2.0, 1.0)};
    const auto motion                    = commanded(via, pair_of(0.0, 0.0));

    CHECK(is_approx_equal(at(*motion, 0.25 * motion->duration()).velocity, pair_of(0.75, 0.375)));
    CHECK(is_approx_equal(at(*motion, 0.75 * motion->duration()).velocity, pair_of(0.75, 0.375)));
    CHECK(is_approx_equal(at(*motion, 0.5 * motion->duration()).velocity, pair_of(1.0, 0.5)));
    CHECK(at(*motion, 0.0).velocity.isZero(default_tolerance));
    CHECK(at(*motion, motion->duration()).velocity.isZero(default_tolerance));
    CHECK(at(*motion, 0.5 * motion->duration()).acceleration.isZero(default_tolerance));
}

TEST_CASE("sampling_a_configuration_space_motion_is_a_pure_function_of_the_time_it_is_given")
{
    const std::vector<configuration> via = {pair_of(2.0, 1.0)};
    const auto motion                    = commanded(via, pair_of(0.0, 0.0));

    CHECK(is_approx_equal(at(*motion, 1.3).position, at(*motion, 1.3).position));
    CHECK(is_approx_equal(at(*motion, 0.4).position, pair_of(328.0 / 3375.0, 164.0 / 3375.0)));
    CHECK(is_approx_equal(at(*motion, -1.0).position, pair_of(0.0, 0.0)));
    CHECK(is_approx_equal(at(*motion, 50.0).position, via.front()));
}

TEST_CASE("a_run_holding_no_position_at_all_is_refused_rather_than_discarded")
{
    const configuration seed = pair_of(0.3, -0.2);

    const auto empty = via_point_schedule({}, seed, bounds());

    REQUIRE(!empty);
    CHECK(empty.error() == refusal::unsupported_input);
}

TEST_CASE("two_configurations_of_different_size_are_refused_rather_than_joined")
{
    const configuration seed             = configuration::Zero(3);
    const std::vector<configuration> via = {pair_of(1.0, 1.0)};

    const auto refused = via_point_schedule(via, seed, bounds());

    REQUIRE(!refused);
    CHECK(refused.error() == refusal::unsupported_input);
}

// Lynch & Park, Modern Robotics, sec. 9.2.2.1: the rest-to-rest cubic s = 3 tau^2 - 2 tau^3.
TEST_CASE("a_single_position_is_the_rest_to_rest_cubic_from_the_seed")
{
    const std::vector<configuration> via = {pair_of(2.0, 1.0)};
    const auto motion                    = commanded(via, pair_of(0.0, 0.0));
    const trajectory_sample sampled      = at(*motion, 0.4 * motion->duration());

    CHECK(is_approx_equal(motion->duration(), 3.0));
    CHECK(is_approx_equal(sampled.position, pair_of(0.704, 0.352)));
    CHECK(is_approx_equal(sampled.velocity, pair_of(0.96, 0.48)));
    CHECK(is_approx_equal(sampled.acceleration, pair_of(4.0 / 15.0, 2.0 / 15.0)));
}

TEST_CASE("a_pair_of_positions_is_the_rest_to_rest_cubic_between_them")
{
    const std::vector<configuration> via = {pair_of(1.0, 1.0), pair_of(2.0, 1.0)};
    const auto motion                    = commanded(via, pair_of(-5.0, -5.0));
    const trajectory_sample sampled      = at(*motion, 0.4 * motion->duration());

    CHECK(is_approx_equal(motion->duration(), 1.5));
    CHECK(is_approx_equal(sampled.position, pair_of(1.352, 1.0)));
    CHECK(is_approx_equal(sampled.velocity, pair_of(0.96, 0.0)));
    CHECK(is_approx_equal(sampled.acceleration, pair_of(8.0 / 15.0, 0.0)));
}
