#include "answered.h"

#include "praxis/trajectory/time_scaling.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <type_traits>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::trajectory;

static_assert(std::is_aggregate_v<time_scaling_ops>);
static_assert(std::is_trivially_copyable_v<time_scaling_ops>);

namespace {

using scaling = expected<scaling_sample, refusal> (*)(double t, double duration);

// s(t) = t/T. The scaling the seam must be able to carry at its simplest: its first derivative is
// 1/T at both ends rather than zero, so the motion starts and stops with a step in velocity.
expected<scaling_sample, refusal> linear(double t, double duration)
{
    return scaling_sample{t / duration, 1.0 / duration, 0.0};
}

void require_endpoint_conditions(scaling shaped_in_time, double duration)
{
    REQUIRE(is_approx_equal(answered(shaped_in_time(0.0, duration)).s, 0.0));
    REQUIRE(is_approx_equal(answered(shaped_in_time(duration, duration)).s, 1.0));
}

}

TEST_CASE("the_time_scaling_slots_default_to_refusing_rather_than_to_a_sample_reading_as_a_rest")
{
    time_scaling_ops ops{};

    REQUIRE(ops.cubic != nullptr);
    REQUIRE(ops.quintic != nullptr);
    REQUIRE(ops.trapezoidal != nullptr);

    CHECK(ops.cubic(0.25, 2.0).error() == refusal::not_implemented);
    CHECK(ops.quintic(0.25, 2.0).error() == refusal::not_implemented);
    CHECK(ops.trapezoidal(0.25, 2.0, 1.0, 4.0).error() == refusal::not_implemented);
}

TEST_CASE("a_scaling_bound_into_the_seam_reports_its_endpoint_conditions_and_both_derivatives")
{
    time_scaling_ops ops{.cubic = &linear};
    double duration = 4.0;

    require_endpoint_conditions(ops.cubic, duration);

    REQUIRE(is_approx_equal(answered(ops.cubic(2.0, duration)).s, 0.5));
    REQUIRE(is_approx_equal(answered(ops.cubic(0.0, duration)).ds, 0.25));
    REQUIRE(is_approx_equal(answered(ops.cubic(duration, duration)).ds, 0.25));
    REQUIRE(is_approx_equal(answered(ops.cubic(2.0, duration)).dds, 0.0));
}

TEST_CASE("a_scaling_bound_into_the_seam_is_monotone_over_its_duration")
{
    time_scaling_ops ops{.cubic = &linear};
    double duration    = 4.0;
    double previous    = -1.0;
    int32_t step_count = 64;

    for(int32_t step = 0; step <= step_count; ++step)
    {
        double s = answered(ops.cubic(duration * step / step_count, duration)).s;
        REQUIRE(s > previous);
        previous = s;
    }
    REQUIRE(is_approx_equal(previous, 1.0));
}

TEST_CASE("assigning_one_time_scaling_slot_leaves_the_other_two_on_their_defaults")
{
    time_scaling_ops ops{.quintic = &linear};
    time_scaling_ops defaults{};

    REQUIRE(ops.quintic == &linear);
    REQUIRE(ops.cubic == defaults.cubic);
    REQUIRE(ops.trapezoidal == defaults.trapezoidal);
}
