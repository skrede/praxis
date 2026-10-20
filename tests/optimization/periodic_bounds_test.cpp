#include "projected/periodic_bounds.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <algorithm>

using namespace praxis::optimization::projected;

namespace {

constexpr double turn      = 2.0 * std::numbers::pi;
constexpr double degree    = std::numbers::pi / 180.0;
constexpr double unbounded = std::numeric_limits<double>::infinity();

bool near(double value, double expected)
{
    return std::abs(value - expected) <= 1.0e-12;
}

// {lower, upper, value, named} in degrees, the period a whole turn.
constexpr std::array<std::array<double, 4>, 18> whole_turns_past_a_bound{{{-120.0, 120.0, 240.0, -120.0},
                                                                          {-120.0, 120.0, 840.0, 120.0},
                                                                          {-120.0, 120.0, -840.0, -120.0},
                                                                          {-135.0, 135.0, 495.0, 135.0},
                                                                          {-135.0, 135.0, -495.0, -135.0},
                                                                          {-99.0, 99.0, 459.0, 99.0},
                                                                          {-99.0, 99.0, -459.0, -99.0},
                                                                          {-93.0, 93.0, 1173.0, 93.0},
                                                                          {-93.0, 93.0, -1173.0, -93.0},
                                                                          {-100.0, 150.0, 510.0, 150.0},
                                                                          {-100.0, 150.0, -460.0, -100.0},
                                                                          {-183.0, 183.0, 1983.0, 183.0},
                                                                          {-183.0, 183.0, -1983.0, -183.0},
                                                                          {-unbounded, 135.0, 495.0, 135.0},
                                                                          {-135.0, unbounded, -495.0, -135.0},
                                                                          {-unbounded, 120.0, 840.0, 120.0},
                                                                          {-120.0, unbounded, -840.0, -120.0},
                                                                          {-unbounded, unbounded, 495.0, 495.0}}};

}

TEST_CASE("a value a whole period past either bound is named inside the bounds")
{
    CHECK(near(projected_between(2.5 + turn, -3.0, 3.0, turn), 2.5));
    CHECK(near(projected_between(-2.5 - turn, -3.0, 3.0, turn), -2.5));
    CHECK(near(projected_between(1.0 - 3.0 * turn, -3.0, 3.0, turn), 1.0));
    CHECK(near(projected_between(3.0 + turn, -3.0, 3.0, turn), 3.0));
    CHECK(near(projected_between(-3.0 - turn, -3.0, 3.0, turn), -3.0));
}

TEST_CASE("a value with no naming inside the bounds takes the bound nearer on the circle, a tie the bound crossed")
{
    CHECK(projected_between(3.1, -3.0, 3.0, turn) == 3.0);
    CHECK(projected_between(3.2, -3.0, 3.0, turn) == -3.0);
    CHECK(projected_between(-3.1, -3.0, 3.0, turn) == -3.0);
    CHECK(projected_between(-3.2, -3.0, 3.0, turn) == 3.0);
    CHECK(projected_between(4.0, -3.0, 3.0, 8.0) == 3.0);
    CHECK(projected_between(-4.0, -3.0, 3.0, 8.0) == -3.0);

    CHECK(near(projected_between(200.0 * degree, -170.0 * degree, 170.0 * degree, turn), -160.0 * degree));
    CHECK(projected_between(185.0 * degree, -170.0 * degree, 170.0 * degree, turn) == -170.0 * degree);
    CHECK(projected_between(180.0 * degree, -170.0 * degree, 170.0 * degree, turn) == 170.0 * degree);
    CHECK(projected_between(-180.0 * degree, -170.0 * degree, 170.0 * degree, turn) == -170.0 * degree);
    CHECK(projected_between(-185.0 * degree, -170.0 * degree, 170.0 * degree, turn) == 170.0 * degree);
}

TEST_CASE("a period of zero is the clamp and an infinite bound leaves the value free")
{
    for(const double value : {0.5, -7.0, 0.1, std::numeric_limits<double>::quiet_NaN()})
    {
        const double clamped   = std::clamp(value, -0.2, 0.3);
        const double projected = projected_between(value, -0.2, 0.3, 0.0);
        CHECK((projected == clamped || (std::isnan(projected) && std::isnan(clamped))));
    }
    CHECK(projected_between(9.0, -unbounded, unbounded, turn) == 9.0);
    CHECK(projected_between(9.0, -unbounded, unbounded, 0.0) == 9.0);
    CHECK_FALSE(named_between(0.5, -0.2, 0.3, 0.0).has_value());
}

TEST_CASE("each value whole periods past a bound is named at its expected value, and the naming projects to itself")
{
    for(const auto &[lower, upper, value, named] : whole_turns_past_a_bound)
    {
        CAPTURE(lower, upper, value, named);
        const double projected                 = projected_between(value * degree, lower * degree, upper * degree, turn);
        const std::optional<double> named_here = named_between(value * degree, lower * degree, upper * degree, turn);
        CHECK(near(projected, named * degree));
        REQUIRE(named_here.has_value());
        CHECK(*named_here == projected);
        CHECK(lower * degree <= projected);
        CHECK(projected <= upper * degree);
        CHECK(projected_between(projected, lower * degree, upper * degree, turn) == projected);
    }
}
