#include "answered.h"

#include "praxis/trajectory/baseline/time_scaling.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <array>
#include <limits>
#include <cstdint>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::trajectory;

namespace {

using scaling = expected<scaling_sample, refusal> (*)(double t, double duration);

void require_runs_from_zero_to_one(scaling shaped_in_time, double duration)
{
    REQUIRE(is_approx_equal(answered(shaped_in_time(0.0, duration)).s, 0.0));
    REQUIRE(is_approx_equal(answered(shaped_in_time(duration, duration)).s, 1.0));

    double previous = -1.0;
    for(int32_t step = 0; step <= 64; ++step)
    {
        const double s = answered(shaped_in_time(duration * step / 64.0, duration)).s;
        REQUIRE(s >= previous);
        previous = s;
    }
}

}

TEST_CASE("every_reference_time_scaling_runs_the_path_parameter_from_zero_to_one")
{
    require_runs_from_zero_to_one(&cubic, 3.0);
    require_runs_from_zero_to_one(&quintic, 3.0);

    CHECK(is_approx_equal(answered(trapezoidal(0.0, 3.0, 1.0, 4.0)).s, 0.0));
    CHECK(is_approx_equal(answered(trapezoidal(3.0, 3.0, 1.0, 4.0)).s, 1.0, 1.0e-9));
}

// Lynch & Park, Modern Robotics, sec. 9.2.2: the cubic holds the endpoint velocities at zero and
// leaves the acceleration stepping at both ends; the quintic holds both.
TEST_CASE("the_cubic_stops_at_both_ends_and_the_quintic_also_arrives_without_acceleration")
{
    const double duration = 2.0;

    CHECK(is_approx_equal(answered(cubic(0.0, duration)).ds, 0.0));
    CHECK(is_approx_equal(answered(cubic(duration, duration)).ds, 0.0));
    CHECK(std::abs(answered(cubic(0.0, duration)).dds) > 1.0);

    CHECK(is_approx_equal(answered(quintic(0.0, duration)).ds, 0.0));
    CHECK(is_approx_equal(answered(quintic(duration, duration)).ds, 0.0));
    CHECK(is_approx_equal(answered(quintic(0.0, duration)).dds, 0.0));
    CHECK(is_approx_equal(answered(quintic(duration, duration)).dds, 0.0));
}

// The peaks a commanded motion derives its duration from: 15/(8T) in the first derivative of the
// quintic and 3/(2T) in that of the cubic, both at the midpoint.
TEST_CASE("the_polynomial_scalings_reach_the_derivative_peaks_a_commanded_duration_is_derived_from")
{
    const double duration = 2.0;

    CHECK(is_approx_equal(answered(quintic(0.5 * duration, duration)).s, 0.5));
    CHECK(is_approx_equal(answered(quintic(0.5 * duration, duration)).ds, 15.0 / (8.0 * duration)));
    CHECK(is_approx_equal(answered(quintic(0.5 * duration, duration)).dds, 0.0));
    CHECK(is_approx_equal(answered(cubic(0.5 * duration, duration)).s, 0.5));
    CHECK(is_approx_equal(answered(cubic(0.5 * duration, duration)).ds, 1.5 / duration));
    CHECK(is_approx_equal(answered(cubic(0.5 * duration, duration)).dds, 0.0));
}

TEST_CASE("the_trapezoidal_cruises_between_its_two_ramps_and_stays_under_the_velocity_bound")
{
    const double duration = 3.0;
    const double cruising = answered(trapezoidal(0.5 * duration, duration, 1.0, 4.0)).ds;

    CHECK(cruising > 0.0);
    CHECK(cruising <= 1.0);
    CHECK(is_approx_equal(answered(trapezoidal(0.5 * duration, duration, 1.0, 4.0)).dds, 0.0));
    CHECK(answered(trapezoidal(0.01 * duration, duration, 1.0, 4.0)).dds > 0.0);
    CHECK(answered(trapezoidal(0.99 * duration, duration, 1.0, 4.0)).dds < 0.0);
}

// A duration or a bound the profile cannot be built or stretched to is a request shape the reference
// does not serve. A sample reading as "at the beginning, at rest" is what a caller cannot tell from
// a motion that genuinely has not started, so no sample is produced at all.
TEST_CASE("a_degenerate_or_infeasible_request_is_refused_rather_than_answered_with_a_sample")
{
    const std::array<expected<scaling_sample, refusal>, 7> refused{
            cubic(0.5, 0.0),
            cubic(0.5, -1.0),
            quintic(0.5, 0.0),
            quintic(0.5, std::numeric_limits<double>::quiet_NaN()),
            trapezoidal(0.5, 3.0, 0.0, 4.0),
            trapezoidal(0.5, 3.0, 1.0, -4.0),
            trapezoidal(0.5, 0.5, 1.0, 4.0),
    };

    for(const expected<scaling_sample, refusal> &sample : refused)
    {
        REQUIRE(!sample);
        CHECK(sample.error() == refusal::unsupported_input);
    }
}
