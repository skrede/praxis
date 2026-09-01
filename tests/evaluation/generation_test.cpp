#include "praxis/evaluation/tolerance.h"
#include "praxis/evaluation/generation.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <algorithm>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr std::uint64_t recorded_seed = 0x5EEDu;
constexpr int draws                   = 10000;
constexpr double half_turn            = std::numbers::pi_v<double>;

// Metres.
constexpr double position_extent = 2.0;

// Metres per radian.
constexpr double pitch_extent = 2.25;

// The ends of the axis-count range the header states.
constexpr std::size_t fewest_axes = 1u;
constexpr std::size_t most_axes   = 8u;

double fraction_of(int count)
{
    return static_cast<double>(count) / static_cast<double>(draws);
}

// Four standard errors of a mean over `draws` samples of the given variance: a band a draw sits
// inside unless the distribution it came from is the wrong one.
double four_errors_of(double variance)
{
    return 4.0 * std::sqrt(variance / static_cast<double>(draws));
}

}

// Archimedes' hat-box theorem: one component of a point drawn uniformly on the sphere is itself
// uniform over the closed interval from minus one to one. A box draw normalized is not, because the
// cube's diagonals stand further from its centre than its faces do.
TEST_CASE("a_drawn_unit_direction_is_uniform_over_the_sphere_rather_than_a_normalized_box_draw")
{
    case_source drawn     = case_source::for_slot(recorded_seed, "unit_direction");
    Eigen::Vector3d total = Eigen::Vector3d::Zero();
    double worst_norm     = 0.0;
    int beyond_half       = 0;

    for(int sample = 0; sample < draws; ++sample)
    {
        const Eigen::Vector3d direction = drawn.unit_direction();

        total += direction;
        worst_norm = std::max(worst_norm, std::fabs(direction.norm() - 1.0));
        beyond_half += std::fabs(direction.z()) > 0.5 ? 1 : 0;
    }

    REQUIRE(worst_norm <= default_tolerance);
    REQUIRE((total / static_cast<double>(draws)).cwiseAbs().maxCoeff() < four_errors_of(1.0 / 3.0));
    REQUIRE(std::fabs(fraction_of(beyond_half) - 0.5) < four_errors_of(0.25));
}

TEST_CASE("a_drawn_position_lies_inside_the_stated_range_and_reaches_both_of_its_ends")
{
    case_source drawn = case_source::for_slot(recorded_seed, "position_metres");
    double furthest   = 0.0;
    double nearest    = position_extent;

    for(int sample = 0; sample < draws; ++sample)
    {
        const Eigen::Vector3d at = drawn.position_metres();

        furthest = std::max(furthest, at.cwiseAbs().maxCoeff());
        nearest  = std::min(nearest, at.cwiseAbs().minCoeff());
    }

    REQUIRE(furthest <= position_extent);
    REQUIRE(furthest > 0.9 * position_extent);
    REQUIRE(nearest < 0.1 * position_extent);
}

TEST_CASE("a_drawn_euler_triple_lies_inside_a_full_turn_and_reaches_its_ends")
{
    case_source drawn = case_source::for_slot(recorded_seed, "euler_triple_radians");
    double furthest   = 0.0;

    for(int sample = 0; sample < draws; ++sample)
        furthest = std::max(furthest, drawn.euler_triple_radians().cwiseAbs().maxCoeff());

    REQUIRE(furthest <= half_turn);
    REQUIRE(furthest > 0.9 * half_turn);
}

TEST_CASE("a_drawn_pitch_straddles_zero_and_reaches_a_travel_that_dominates_the_rotation")
{
    case_source drawn = case_source::for_slot(recorded_seed, "pitch");
    double furthest   = 0.0;
    double nearest    = pitch_extent;
    int negative      = 0;

    for(int sample = 0; sample < draws; ++sample)
    {
        const double travel_per_radian = drawn.pitch();

        furthest = std::max(furthest, std::fabs(travel_per_radian));
        nearest  = std::min(nearest, std::fabs(travel_per_radian));
        negative += travel_per_radian < 0.0 ? 1 : 0;
    }

    REQUIRE(furthest <= pitch_extent);
    REQUIRE(furthest > 1.0);
    REQUIRE(nearest < 0.01);
    REQUIRE(std::fabs(fraction_of(negative) - 0.5) < four_errors_of(0.25));
}

TEST_CASE("a_drawn_angular_part_is_exactly_zero_half_the_time_so_a_motion_with_no_rotation_is_reached")
{
    case_source drawn = case_source::for_slot(recorded_seed, "angular_part");
    int silent        = 0;

    for(int sample = 0; sample < draws; ++sample)
        silent += drawn.angular_part() == Eigen::Vector3d::Zero() ? 1 : 0;

    REQUIRE(std::fabs(fraction_of(silent) - 0.5) < four_errors_of(0.25));
}

TEST_CASE("a_drawn_linear_part_and_a_drawn_normal_triple_are_unconstrained_and_finite")
{
    case_source drawn  = case_source::for_slot(recorded_seed, "linear_part");
    double furthest    = 0.0;
    bool every_finite  = true;
    bool ever_negative = false;

    for(int sample = 0; sample < draws; ++sample)
    {
        const Eigen::Vector3d part          = drawn.linear_part();
        const Eigen::Vector3d unconstrained = drawn.normal_triple();

        furthest      = std::max({furthest, part.cwiseAbs().maxCoeff(), unconstrained.cwiseAbs().maxCoeff()});
        every_finite  = every_finite && part.allFinite() && unconstrained.allFinite();
        ever_negative = ever_negative || part.minCoeff() < 0.0;
    }

    REQUIRE(every_finite);
    REQUIRE(ever_negative);
    REQUIRE(furthest > 1.0);
}

TEST_CASE("a_drawn_rotation_order_index_answers_every_ordering_and_none_outside_the_range")
{
    case_source drawn = case_source::for_slot(recorded_seed, "axis_order_index");
    std::array<int, 12> seen{};
    bool every_inside = true;

    for(int sample = 0; sample < draws; ++sample)
    {
        const std::size_t index = drawn.axis_order_index();

        every_inside = every_inside && index < seen.size();
        seen[std::min(index, seen.size() - 1u)] += 1;
    }

    REQUIRE(every_inside);
    REQUIRE(std::count(seen.begin(), seen.end(), 0) == 0);
}

TEST_CASE("a_drawn_axis_count_answers_every_length_the_range_admits_and_none_outside_it")
{
    case_source drawn = case_source::for_slot(recorded_seed, "axis_count");
    std::array<int, most_axes - fewest_axes + 1u> seen{};
    bool every_inside = true;

    for(int sample = 0; sample < draws; ++sample)
    {
        const std::size_t count = drawn.axis_count();

        every_inside = every_inside && count >= fewest_axes && count <= most_axes;
        seen[std::min(count - fewest_axes, seen.size() - 1u)] += 1;
    }

    REQUIRE(every_inside);
    REQUIRE(std::count(seen.begin(), seen.end(), 0) == 0);
}
