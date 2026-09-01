#include "captured_log.h"
#include "configuration_run.h"

#include "praxis/trajectory/baseline/trajectory.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <cmath>
#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::trajectory;

namespace {

constexpr double one_sided_step        = 1.0e-9;
constexpr double via_point_tolerance   = 1.0e-9;
constexpr double continuity_tolerance  = 1.0e-6;
constexpr double apportioned_traversal = 4.5;

// The apportionment the reference is held to, written out here rather than taken from it, so a
// change to either is a failure rather than a silent agreement. Every segment carries the same share
// of whatever the run was stretched to.
std::vector<double> knots(std::span<const configuration> via, double stretched_to)
{
    const configuration velocity = bounds().velocity;
    std::vector<double> times    = {0.0};

    for(std::size_t k = 1u; k < via.size(); ++k)
    {
        double span = 0.0;
        for(Eigen::Index i = 0; i < via[k].size(); ++i)
            span = std::max(span, std::abs(via[k][i] - via[k - 1u][i]) / velocity[i]);

        times.push_back(times.back() + span);
    }

    for(double &knot : times)
        knot *= stretched_to / times.back();

    return times;
}

std::vector<configuration> four_via_points()
{
    return {pair_of(0.0, 0.0), pair_of(1.0, 0.5), pair_of(0.5, 1.5), pair_of(2.0, 1.0)};
}

double fastest_reached(const trajectory_generator &motion, std::size_t steps)
{
    const configuration velocity = bounds().velocity;
    double reached               = 0.0;

    for(std::size_t step = 0u; step <= steps; ++step)
    {
        const trajectory_sample sampled = at(motion, motion.duration() * static_cast<double>(step) / static_cast<double>(steps));
        for(Eigen::Index i = 0; i < velocity.size(); ++i)
            reached = std::max(reached, std::abs(sampled.velocity[i]) / velocity[i]);
    }

    return reached;
}

}

TEST_CASE("a_run_of_via_points_stands_at_each_of_them_at_its_own_knot_time")
{
    const std::vector<configuration> via = four_via_points();
    const auto motion                    = commanded(via, pair_of(-5.0, -5.0));
    const std::vector<double> times      = knots(via, motion->duration());

    for(std::size_t k = 0u; k < via.size(); ++k)
        CHECK(is_approx_equal(at(*motion, times[k]).position, via[k], via_point_tolerance));
}

// The apportionment gives each segment the time its slowest degree of freedom needs at its bound,
// which is the mean rate; a cubic through a via point passes it at speed and so reaches more. The
// run is stretched until the largest rate it reaches is the bound itself.
TEST_CASE("a_run_of_via_points_reaches_a_velocity_bound_and_crosses_none_of_them")
{
    const std::vector<configuration> via = four_via_points();
    const auto motion                    = commanded(via, pair_of(-5.0, -5.0));
    const double reached                 = fastest_reached(*motion, 20000u);

    CHECK(motion->duration() > apportioned_traversal);
    CHECK(reached <= 1.0 + continuity_tolerance);
    CHECK(reached >= 1.0 - continuity_tolerance);
}

TEST_CASE("the_velocity_a_run_of_via_points_reports_is_continuous_across_every_interior_via_point")
{
    const std::vector<configuration> via = four_via_points();
    const auto motion                    = commanded(via, pair_of(-5.0, -5.0));
    const std::vector<double> times      = knots(via, motion->duration());

    for(std::size_t k = 1u; k + 1u < times.size(); ++k)
        CHECK(is_approx_equal(at(*motion, times[k] - one_sided_step).velocity, at(*motion, times[k] + one_sided_step).velocity, continuity_tolerance));
}

TEST_CASE("the_acceleration_a_run_of_via_points_reports_is_continuous_across_every_interior_via_point")
{
    const std::vector<configuration> via = four_via_points();
    const auto motion                    = commanded(via, pair_of(-5.0, -5.0));
    const std::vector<double> times      = knots(via, motion->duration());

    for(std::size_t k = 1u; k + 1u < times.size(); ++k)
        CHECK(is_approx_equal(at(*motion, times[k] - one_sided_step).acceleration, at(*motion, times[k] + one_sided_step).acceleration, continuity_tolerance));
}

TEST_CASE("a_run_of_via_points_begins_and_ends_at_rest_in_every_degree_of_freedom")
{
    const std::vector<configuration> via = four_via_points();
    const auto motion                    = commanded(via, pair_of(-5.0, -5.0));

    CHECK(at(*motion, 0.0).velocity.isZero(via_point_tolerance));
    CHECK(at(*motion, motion->duration()).velocity.isZero(via_point_tolerance));
}

TEST_CASE("a_run_of_three_waypoints_runs_between_them_and_not_from_the_seed")
{
    const std::vector<configuration> via = {pair_of(1.0, 1.0), pair_of(2.0, 1.0), pair_of(3.0, 2.0)};
    const auto motion                    = commanded(via, pair_of(-5.0, -5.0));

    CHECK(is_approx_equal(at(*motion, 0.0).position, via.front(), via_point_tolerance));
    CHECK(is_approx_equal(at(*motion, motion->duration()).position, via.back(), via_point_tolerance));
}

TEST_CASE("a_run_repeating_the_configuration_it_already_stands_at_is_refused_and_the_row_is_named")
{
    const std::vector<configuration> via = {pair_of(1.0, 1.0), pair_of(2.0, 1.0), pair_of(2.0, 1.0), pair_of(3.0, 2.0)};

    praxis::tests::captured_log captured;
    const auto refused         = joint_space_waypoints(via, pair_of(-5.0, -5.0), bounds());
    const std::string reported = captured.text();

    REQUIRE(!refused);
    CHECK(refused.error() == refusal::unsupported_input);
    CHECK(reported.find("row 2") != std::string::npos);
}

TEST_CASE("a_run_of_via_points_of_differing_widths_is_refused_rather_than_joined")
{
    const std::vector<configuration> via = {pair_of(1.0, 1.0), configuration::Zero(3), pair_of(3.0, 2.0)};

    const auto refused = joint_space_waypoints(via, pair_of(-5.0, -5.0), bounds());

    REQUIRE(!refused);
    CHECK(refused.error() == refusal::unsupported_input);
}
