#include "praxis/trajectory/detail/finite_difference.h"

#include "praxis/rigid_motion/types.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <array>
#include <algorithm>

using namespace praxis;

namespace {

// A quadratic is the lowest degree at which the second divided difference is neither exact by
// construction nor identically zero, so it discriminates both terms of the stencil at once.
constexpr double first_agreement  = 1.0e-9;
constexpr double second_agreement = 1.0e-6;

double quadratic(double s)
{
    return 3.0 * s * s - 2.0 * s + 1.0;
}

expected<double, refusal> sampled_quadratic(double s)
{
    return quadratic(s);
}

expected<twist, refusal> curved(double s)
{
    twist coordinates;
    coordinates << quadratic(s), -0.5 * s * s, 2.0 * s, 1.0, s * s + s, 4.0 - 3.0 * s * s;

    return coordinates;
}

expected<twist, refusal> refusing(double)
{
    return praxis::unexpected(refusal::degenerate);
}

twist curved_slope(double s)
{
    twist rate;
    rate << 6.0 * s - 2.0, -s, 2.0, 0.0, 2.0 * s + 1.0, -6.0 * s;

    return rate;
}

twist curved_bend()
{
    twist rate;
    rate << 6.0, -1.0, 0.0, 0.0, 2.0, -6.0;

    return rate;
}

}

TEST_CASE("the_stencil_reproduces_both_derivatives_of_a_quadratic_in_the_path_parameter")
{
    const auto along = trajectory::detail::central_differences(&sampled_quadratic, 0.4);

    REQUIRE(along);
    CHECK(std::abs(along->first_derivative - 0.4) < first_agreement);
    CHECK(std::abs(along->second_derivative - 6.0) < second_agreement);
}

TEST_CASE("the_stencil_carries_a_second_sampled_type_through_the_same_arithmetic")
{
    const auto along = trajectory::detail::central_differences(&curved, 0.4);

    REQUIRE(along);
    CHECK((along->first_derivative - curved_slope(0.4)).norm() < first_agreement);
    CHECK((along->second_derivative - curved_bend()).norm() < second_agreement);
}

TEST_CASE("the_stencil_evaluates_the_sampler_only_inside_the_closed_unit_interval")
{
    for(double s : std::array<double, 2>{0.0, 1.0})
    {
        double lowest      = 2.0;
        double highest     = -1.0;
        const auto watched = [&lowest, &highest](double u) -> expected<double, refusal>
        {
            lowest  = std::min(lowest, u);
            highest = std::max(highest, u);

            return quadratic(u);
        };

        const auto along = trajectory::detail::central_differences(watched, s);

        REQUIRE(along);
        CHECK(std::isfinite(along->first_derivative));
        CHECK(lowest >= 0.0);
        CHECK(highest <= 1.0);
    }
}

// A derivative formed over a substituted value would be indistinguishable from one taken over a
// path the sampler could actually answer for, so the refusal has to reach the stencil's caller.
TEST_CASE("a_sampler_that_refuses_refuses_the_difference_rather_than_being_differenced")
{
    const auto refused = trajectory::detail::central_differences(&refusing, 0.4);

    REQUIRE(!refused);
    CHECK(refused.error() == refusal::degenerate);
}
