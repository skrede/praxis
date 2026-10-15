#include "captured_log.h"
#include "configuration_run.h"

#include "praxis/trajectory/baseline/trajectory.h"
#include "praxis/trajectory/baseline/time_scaling.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::trajectory;

namespace {

constexpr double via_point_tolerance  = 1.0e-12;
constexpr double one_sided_step       = 1.0e-9;
constexpr double continuity_tolerance = 1.0e-6;

// Lynch & Park, Modern Robotics, sec. 9.3, Fig. 9.8.
std::vector<via_point> figure_9_8()
{
    return {via_point{0.0, pair_of(0.0, 0.0), pair_of(0.0, 0.0)}, via_point{1.0, pair_of(0.0, 1.0), pair_of(1.0, 0.0)}, via_point{2.0, pair_of(1.0, 1.0), pair_of(0.0, -1.0)},
            via_point{3.0, pair_of(1.0, 0.0), pair_of(0.0, 0.0)}};
}

std::size_t error_lines(const std::string &text)
{
    std::size_t lines = 0;
    for(std::size_t at = text.find("[error]"); at != std::string::npos; at = text.find("[error]", at + 1u))
        ++lines;

    return lines;
}

void require_refused_with_one_line(const std::vector<via_point> &via)
{
    expected<std::unique_ptr<trajectory_generator>, refusal> answered = std::unique_ptr<trajectory_generator>();
    const std::string text                                            = tests::reported_by([&] { answered = joint_space_via_points(via); });

    REQUIRE_FALSE(answered);
    REQUIRE(answered.error() == refusal::unsupported_input);
    REQUIRE(error_lines(text) == 1u);
}

}

TEST_CASE("the_book_example_passes_through_every_via_point_at_its_time_with_its_velocity")
{
    const std::vector<via_point> via = figure_9_8();
    const auto motion                = joined(via);

    REQUIRE(motion->duration() == 3.0);
    for(const via_point &point : via)
    {
        const trajectory_sample sampled = at(*motion, point.time);

        INFO("t = " << point.time);
        CHECK(is_approx_equal(sampled.position, point.position, via_point_tolerance));
        CHECK(is_approx_equal(sampled.velocity, point.velocity, via_point_tolerance));
    }
}

TEST_CASE("the_book_example_arrives_at_each_interior_via_point_with_the_velocity_it_leaves_it_at")
{
    const std::vector<via_point> via = figure_9_8();
    const auto motion                = joined(via);

    for(std::size_t k = 1u; k + 1u < via.size(); ++k)
        CHECK(is_approx_equal(at(*motion, via[k].time - one_sided_step).velocity, via[k].velocity, continuity_tolerance));
}

TEST_CASE("two_via_points_at_rest_are_joined_by_the_cubic_time_scaling_of_the_straight_line_between_them")
{
    const double T           = 1.7;
    const configuration q0   = pair_of(0.3, -1.2);
    const configuration q1   = pair_of(1.1, 0.4);
    const configuration rest = pair_of(0.0, 0.0);
    const std::vector<via_point> via{via_point{0.0, q0, rest}, via_point{T, q1, rest}};
    const auto motion = joined(via);

    for(std::size_t k = 0u; k <= 32u; ++k)
    {
        const double t                  = T * static_cast<double>(k) / 32.0;
        const auto scaled               = cubic(t, T);
        const trajectory_sample sampled = at(*motion, t);
        REQUIRE(scaled);

        INFO("t = " << t);
        CHECK(is_approx_equal(sampled.position, configuration(q0 + scaled->s * (q1 - q0)), via_point_tolerance));
        CHECK(is_approx_equal(sampled.velocity, configuration(scaled->ds * (q1 - q0)), via_point_tolerance));
        CHECK(is_approx_equal(sampled.acceleration, configuration(scaled->dds * (q1 - q0)), via_point_tolerance));
    }
}

TEST_CASE("a_run_of_via_points_outside_what_the_slot_joins_is_refused_with_one_error_line")
{
    const double unknown                    = std::numeric_limits<double>::quiet_NaN();
    std::vector<via_point> late             = figure_9_8();
    late.front().time                       = 0.5;
    std::vector<via_point> repeated         = figure_9_8();
    repeated[2].time                        = 1.0;
    std::vector<via_point> unknown_time     = figure_9_8();
    unknown_time[1].time                    = unknown;
    std::vector<via_point> wider            = figure_9_8();
    wider[2].velocity                       = configuration::Zero(3);
    std::vector<via_point> unknown_position = figure_9_8();
    unknown_position[3].position[1]         = unknown;

    require_refused_with_one_line({});
    require_refused_with_one_line({figure_9_8().front()});
    require_refused_with_one_line(late);
    require_refused_with_one_line(repeated);
    require_refused_with_one_line(unknown_time);
    require_refused_with_one_line(wider);
    require_refused_with_one_line(unknown_position);
}
