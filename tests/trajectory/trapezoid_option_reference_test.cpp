#include "answered.h"

#include "praxis/trajectory/baseline/time_scaling.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <array>
#include <limits>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::trajectory;

namespace {

constexpr double not_a_number = std::numeric_limits<double>::quiet_NaN();

void require_refused(const expected<scaling_sample, refusal> &sample)
{
    REQUIRE(!sample);
    CHECK(sample.error() == refusal::unsupported_input);
}

}

TEST_CASE("choosing_v_and_a_reaches_one_at_the_duration_eq_9_24_solves_for")
{
    const double duration = (4.0 + 1.0 * 1.0) / (1.0 * 4.0);

    CHECK(duration == 1.25);
    CHECK(std::abs(answered(trapezoidal_from_velocity_acceleration(duration, 1.0, 4.0)).s - 1.0) <= 1.0e-15);
    CHECK(answered(trapezoidal_from_velocity_acceleration(0.5 * duration, 1.0, 4.0)).ds == 1.0);
    CHECK(answered(trapezoidal_from_velocity_acceleration(0.1, 1.0, 4.0)).ds == 4.0 * 0.1);
    CHECK(answered(trapezoidal_from_velocity_acceleration(0.1, 1.0, 4.0)).dds == 4.0);
}

TEST_CASE("choosing_v_and_a_holds_its_end_values_outside_the_duration")
{
    const scaling_sample before = answered(trapezoidal_from_velocity_acceleration(-0.5, 1.0, 4.0));
    const scaling_sample after  = answered(trapezoidal_from_velocity_acceleration(3.0, 1.0, 4.0));
    const scaling_sample start  = answered(trapezoidal_from_velocity_acceleration(0.0, 1.0, 4.0));
    const scaling_sample finish = answered(trapezoidal_from_velocity_acceleration(1.25, 1.0, 4.0));

    CHECK(before.s == start.s);
    CHECK(before.ds == start.ds);
    CHECK(before.dds == start.dds);
    CHECK(after.s == finish.s);
    CHECK(after.ds == finish.ds);
    CHECK(after.dds == finish.dds);
}

TEST_CASE("choosing_v_and_a_refuses_outside_the_books_domain_and_any_non_finite_or_non_positive_input")
{
    const std::array refused{
            trapezoidal_from_velocity_acceleration(0.5, 3.0, 4.0),          trapezoidal_from_velocity_acceleration(0.5, 0.0, 4.0),
            trapezoidal_from_velocity_acceleration(0.5, 1.0, -1.0),         trapezoidal_from_velocity_acceleration(0.5, not_a_number, 4.0),
            trapezoidal_from_velocity_acceleration(0.5, 1.0, not_a_number), trapezoidal_from_velocity_acceleration(0.5, 1.0, std::numeric_limits<double>::infinity()),
    };

    for(const expected<scaling_sample, refusal> &sample : refused)
        require_refused(sample);
}

TEST_CASE("choosing_v_and_t_solves_the_acceleration_and_reaches_one_at_the_duration")
{
    CHECK(answered(trapezoidal_from_velocity_duration(0.1, 1.5, 1.0)).dds == 1.0 / (1.5 - 1.0));
    CHECK(std::abs(answered(trapezoidal_from_velocity_duration(1.5, 1.5, 1.0)).s - 1.0) <= 1.0e-15);
    CHECK(answered(trapezoidal_from_velocity_duration(0.75, 1.5, 1.0)).ds == 1.0);
}

TEST_CASE("choosing_v_and_t_answers_the_triangle_at_vt_two_and_refuses_outside_the_books_domain")
{
    CHECK(std::abs(answered(trapezoidal_from_velocity_duration(1.0, 2.0, 1.0)).ds - 1.0) <= 1.0e-15);
    CHECK(std::abs(answered(trapezoidal_from_velocity_duration(2.0, 2.0, 1.0)).s - 1.0) <= 1.0e-15);

    const std::array refused{
            trapezoidal_from_velocity_duration(0.5, 1.0, 1.0),          trapezoidal_from_velocity_duration(0.5, 2.5, 1.0),
            trapezoidal_from_velocity_duration(0.5, 0.0, 1.0),          trapezoidal_from_velocity_duration(0.5, 1.5, -1.0),
            trapezoidal_from_velocity_duration(0.5, not_a_number, 1.0), trapezoidal_from_velocity_duration(0.5, 1.5, std::numeric_limits<double>::infinity()),
    };

    for(const expected<scaling_sample, refusal> &sample : refused)
        require_refused(sample);
}

TEST_CASE("choosing_a_and_t_solves_the_cruise_rate_and_reaches_one_at_the_duration")
{
    CHECK(std::abs(answered(trapezoidal_from_acceleration_duration(0.625, 1.25, 4.0)).ds - 1.0) <= 1.0e-15);
    CHECK(std::abs(answered(trapezoidal_from_acceleration_duration(1.25, 1.25, 4.0)).s - 1.0) <= 1.0e-15);
    CHECK(answered(trapezoidal_from_acceleration_duration(0.1, 1.25, 4.0)).dds == 4.0);
}

TEST_CASE("choosing_a_and_t_answers_the_triangle_at_at_squared_four_and_refuses_outside_the_books_domain")
{
    CHECK(answered(trapezoidal_from_acceleration_duration(0.5, 1.0, 4.0)).ds == std::sqrt(4.0));

    const std::array refused{
            trapezoidal_from_acceleration_duration(0.5, 0.9, 4.0),
            trapezoidal_from_acceleration_duration(0.5, 0.0, 4.0),
            trapezoidal_from_acceleration_duration(0.5, 1.25, -4.0),
            trapezoidal_from_acceleration_duration(0.5, not_a_number, 4.0),
            trapezoidal_from_acceleration_duration(0.5, std::numeric_limits<double>::infinity(), 4.0),
            trapezoidal_from_acceleration_duration(0.5, 1.25, not_a_number),
    };

    for(const expected<scaling_sample, refusal> &sample : refused)
        require_refused(sample);
}

TEST_CASE("the_three_options_agree_on_one_profile")
{
    const double velocity     = 0.8;
    const double acceleration = 2.0;
    const double duration     = (acceleration + velocity * velocity) / (velocity * acceleration);

    CHECK(std::abs(answered(trapezoidal_from_velocity_duration(0.0, duration, velocity)).dds - acceleration) <= 1.0e-14);
    CHECK(std::abs(answered(trapezoidal_from_acceleration_duration(0.5 * duration, duration, acceleration)).ds - velocity) <= 1.0e-14);

    for(int step = 0; step <= 64; ++step)
    {
        const double at                  = duration * step / 64.0;
        const scaling_sample chosen      = answered(trapezoidal_from_velocity_acceleration(at, velocity, acceleration));
        const scaling_sample timed       = answered(trapezoidal_from_velocity_duration(at, duration, velocity));
        const scaling_sample accelerated = answered(trapezoidal_from_acceleration_duration(at, duration, acceleration));

        INFO("step " << step);
        for(const scaling_sample &other : {timed, accelerated})
        {
            CHECK(std::abs(other.s - chosen.s) <= 1.0e-13);
            CHECK(std::abs(other.ds - chosen.ds) <= 1.0e-13);
            CHECK(std::abs(other.dds - chosen.dds) <= 1.0e-13);
        }
    }
}
