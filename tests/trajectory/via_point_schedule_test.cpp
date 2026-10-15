#include "captured_log.h"
#include "evaluation_cases.h"
#include "configuration_run.h"

#include "praxis/trajectory/baseline/trajectory.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/tolerance.h"
#include "praxis/evaluation/generation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::trajectory;

namespace {

constexpr double exact_tolerance          = 1.0e-12;
constexpr double duration_bound           = 1.0e-8;
constexpr double peak_bound               = 1.0e-14;
constexpr std::size_t short_run_cases     = 4000u;
constexpr std::size_t three_or_more_cases = 2000u;
constexpr std::size_t fewest_short_runs   = 500u;
constexpr std::size_t peak_samples        = 4096u;

joint_waypoint_case drawn_run(std::size_t index)
{
    evaluation::case_source drawn = evaluation::case_source::at_case(evaluation::default_seed, "trajectory.via_point_schedule", evaluation::spread::bulk, index);

    return drawn_joint_waypoint_case(drawn);
}

double slowest_traversal(const configuration &from, const configuration &to, const configuration &velocity)
{
    double span = 0.0;
    for(Eigen::Index i = 0; i < from.size(); ++i)
        span = std::max(span, std::abs(to[i] - from[i]) / velocity[i]);

    return span;
}

double fastest_over_bound(const trajectory_generator &motion, const configuration &velocity, std::size_t steps)
{
    double fastest = 0.0;
    for(std::size_t step = 0u; step <= steps; ++step)
    {
        const double t = steps == 0u ? 0.5 * motion.duration() : motion.duration() * static_cast<double>(step) / static_cast<double>(steps);
        fastest        = std::max(fastest, (at(motion, t).velocity.cwiseAbs().array() / velocity.array()).maxCoeff());
    }

    return fastest;
}

// At rest at both via points, 1.5 times the slowest joint's traversal, from and to where it starts
// and ends, and no faster than any joint's bound.
void require_rest_to_rest(const joint_waypoint_case &run)
{
    const configuration &from        = run.waypoints.size() == 1u ? run.seed : run.waypoints.front();
    const std::vector<via_point> via = scheduled(run.waypoints, run.seed, run.limits);
    const auto motion                = joined(via);

    REQUIRE(via.size() == 2u);
    REQUIRE((via.front().velocity.isZero(0.0) && via.back().velocity.isZero(0.0)));
    REQUIRE(std::abs(motion->duration() / (1.5 * slowest_traversal(from, run.waypoints.back(), run.limits.velocity)) - 1.0) <= duration_bound);
    REQUIRE(is_approx_equal(at(*motion, 0.0).position, from, exact_tolerance));
    REQUIRE(is_approx_equal(at(*motion, motion->duration()).position, run.waypoints.back(), exact_tolerance));
    REQUIRE(fastest_over_bound(*motion, run.limits.velocity, 0u) <= 1.0 + peak_bound);
    REQUIRE(fastest_over_bound(*motion, run.limits.velocity, peak_samples) <= 1.0 + peak_bound);
}

void require_one_refusal(std::span<const configuration> positions, const configuration &j0, const configuration_limits &limits)
{
    expected<std::vector<via_point>, refusal> answered = std::vector<via_point>();
    const std::string text                             = tests::reported_by([&] { answered = via_point_schedule(positions, j0, limits); });

    REQUIRE_FALSE(answered);
    REQUIRE(answered.error() == refusal::unsupported_input);
    REQUIRE(text.find("[error]") != std::string::npos);
    REQUIRE(text.find("[error]") == text.rfind("[error]"));
}

}

TEST_CASE("one_configuration_is_reached_from_the_start_at_rest_at_both_ends_with_each_joint_at_its_bound_halfway")
{
    const std::vector<configuration> one = {pair_of(2.0, 1.0)};
    const std::vector<via_point> via     = scheduled(one, pair_of(0.0, 0.0), bounds());

    REQUIRE(via.size() == 2u);
    REQUIRE(via.front().time == 0.0);
    REQUIRE(std::abs(via.back().time / 3.0 - 1.0) <= duration_bound);
    REQUIRE(via.front().position == pair_of(0.0, 0.0));
    REQUIRE(via.back().position == pair_of(2.0, 1.0));

    const auto motion = joined(via);
    const double T    = motion->duration();
    CHECK(is_approx_equal(at(*motion, 0.5 * T).position, pair_of(1.0, 0.5), exact_tolerance));
    CHECK(is_approx_equal(at(*motion, 0.5 * T).velocity, pair_of(1.0, 0.5), exact_tolerance));
    CHECK(is_approx_equal(at(*motion, 0.5 * T).acceleration, pair_of(0.0, 0.0), exact_tolerance));
    CHECK(is_approx_equal(at(*motion, 0.4 * T).position, pair_of(0.704, 0.352), exact_tolerance));
    CHECK(is_approx_equal(at(*motion, 0.4 * T).velocity, pair_of(0.96, 0.48), exact_tolerance));
    CHECK(is_approx_equal(at(*motion, 0.4 * T).acceleration, pair_of(4.0 / 15.0, 2.0 / 15.0), exact_tolerance));
    CHECK(is_approx_equal(at(*motion, 0.0).velocity, pair_of(0.0, 0.0), exact_tolerance));
    CHECK(is_approx_equal(at(*motion, T).velocity, pair_of(0.0, 0.0), exact_tolerance));
}

TEST_CASE("two_configurations_are_joined_at_rest_at_both_from_the_first_whatever_the_start")
{
    const std::vector<configuration> two = {pair_of(1.0, 1.0), pair_of(2.0, 1.0)};
    const auto motion                    = joined(scheduled(two, pair_of(-5.0, -5.0), bounds()));
    const double T                       = motion->duration();

    REQUIRE(std::abs(T / 1.5 - 1.0) <= duration_bound);
    CHECK(is_approx_equal(at(*motion, 0.0).position, pair_of(1.0, 1.0), exact_tolerance));
    CHECK(is_approx_equal(at(*motion, 0.5 * T).position, pair_of(1.5, 1.0), exact_tolerance));
    CHECK(is_approx_equal(at(*motion, 0.5 * T).velocity, pair_of(1.0, 0.0), exact_tolerance));
    CHECK(is_approx_equal(at(*motion, 0.4 * T).position, pair_of(1.352, 1.0), exact_tolerance));
    CHECK(is_approx_equal(at(*motion, 0.4 * T).velocity, pair_of(0.96, 0.0), exact_tolerance));
}

TEST_CASE("every_drawn_run_of_one_or_two_takes_one_and_a_half_times_its_slowest_joint_s_traversal_and_no_joint_passes_its_bound")
{
    std::size_t judged = 0u;
    for(std::size_t index = 0u; index < short_run_cases; ++index)
    {
        const joint_waypoint_case run = drawn_run(index);
        if(run.waypoints.size() > 2u)
            continue;

        INFO("case " << index);
        require_rest_to_rest(run);
        ++judged;
    }

    REQUIRE(judged >= fewest_short_runs);
}

TEST_CASE("a_schedule_of_three_or_more_starts_at_rest_at_zero_ends_at_rest_and_passes_each_interior_row_at_the_book_velocity")
{
    for(std::size_t index = 0u; index < three_or_more_cases; ++index)
    {
        const joint_waypoint_case run = drawn_run(index);
        if(run.waypoints.size() < 3u)
            continue;

        const std::vector<via_point> via = scheduled(run.waypoints, run.seed, run.limits);
        std::vector<double> times;
        for(const via_point &point : via)
            times.push_back(point.time);

        INFO("case " << index);
        REQUIRE(via.front().time == 0.0);
        REQUIRE(std::adjacent_find(times.begin(), times.end(), [](double before, double after) { return !(after > before); }) == times.end());
        REQUIRE(via.front().velocity.isZero(0.0));
        REQUIRE(via.back().velocity.isZero(0.0));
        for(std::size_t k = 1u; k + 1u < via.size(); ++k)
            REQUIRE(is_approx_equal(via[k].velocity, book_velocity(run.waypoints, times, k), exact_tolerance));
    }
}

TEST_CASE("a_run_the_schedule_cannot_time_is_refused_with_one_error_line")
{
    const double unknown            = std::numeric_limits<double>::quiet_NaN();
    const configuration j0          = pair_of(0.5, -0.5);
    configuration_limits held_still = bounds();
    held_still.velocity             = pair_of(1.0, 0.0);
    configuration wider(3);
    wider << 1.0, 2.0, 3.0;

    require_one_refusal(std::vector<configuration>{j0}, j0, bounds());
    require_one_refusal(std::vector<configuration>{pair_of(1.0, 1.0), pair_of(1.0, 1.0)}, j0, bounds());
    require_one_refusal(std::vector<configuration>{pair_of(0.5, 2.0)}, j0, held_still);
    require_one_refusal(std::vector<configuration>{}, j0, bounds());
    require_one_refusal(std::vector<configuration>{wider}, j0, bounds());
    require_one_refusal(std::vector<configuration>{pair_of(1.0, 1.0), wider}, j0, bounds());
    require_one_refusal(std::vector<configuration>{pair_of(1.0, unknown), pair_of(2.0, 1.0)}, j0, bounds());
    require_one_refusal(std::vector<configuration>{pair_of(1.0, 1.0), pair_of(2.0, 1.0), pair_of(2.0, 1.0)}, j0, bounds());
}
