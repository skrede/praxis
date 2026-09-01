#include "praxis/evaluation/tolerance.h"
#include "praxis/evaluation/generation.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <span>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <algorithm>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr std::uint64_t recorded_seed = 0x5EEDu;
constexpr int draws                   = 10000;
constexpr double half_turn            = std::numbers::pi_v<double>;
constexpr double quarter_turn         = 0.5 * std::numbers::pi_v<double>;

// Metres.
constexpr double position_extent = 2.0;

// The widest a near-singular draw stands from the value it crowds.
constexpr double crowding_extent = 1.0;

double nearest_of(double value, std::span<const double> singular)
{
    double nearest = crowding_extent + half_turn;

    for(double at : singular)
        nearest = std::min(nearest, std::fabs(value - at));

    return nearest;
}

}

TEST_CASE("a_near_singular_angle_crowds_zero_and_a_half_turn_while_the_bulk_reaches_between_them")
{
    constexpr std::array<double, 3> singular{0.0, half_turn, -half_turn};
    case_source crowded    = case_source::for_slot(recorded_seed, "angle_radians", spread::near_singular);
    case_source spread_out = case_source::for_slot(recorded_seed, "angle_radians");
    double furthest        = 0.0;
    double nearest         = crowding_extent;
    double midway          = 0.0;

    for(int sample = 0; sample < draws; ++sample)
    {
        const double stands = nearest_of(crowded.angle_radians(), singular);

        furthest = std::max(furthest, stands);
        nearest  = std::min(nearest, stands);
        midway   = std::max(midway, nearest_of(spread_out.angle_radians(), singular));
    }

    REQUIRE(furthest <= crowding_extent);
    REQUIRE(nearest >= default_tolerance);
    REQUIRE(nearest < std::sqrt(default_tolerance));
    REQUIRE(midway > crowding_extent);
}

TEST_CASE("a_near_singular_pitch_crowds_zero_and_never_reaches_it")
{
    case_source crowded = case_source::for_slot(recorded_seed, "pitch", spread::near_singular);
    double furthest     = 0.0;
    double nearest      = crowding_extent;

    for(int sample = 0; sample < draws; ++sample)
    {
        const double travel_per_radian = std::fabs(crowded.pitch());

        furthest = std::max(furthest, travel_per_radian);
        nearest  = std::min(nearest, travel_per_radian);
    }

    REQUIRE(furthest <= crowding_extent);
    REQUIRE(nearest >= default_tolerance);
    REQUIRE(nearest < std::sqrt(default_tolerance));
}

TEST_CASE("a_near_singular_euler_triple_crowds_only_its_middle_angle_against_a_value_that_loses_a_degree_of_freedom")
{
    constexpr std::array<double, 4> singular{0.0, quarter_turn, -quarter_turn, half_turn};
    case_source crowded = case_source::for_slot(recorded_seed, "euler_triple_radians", spread::near_singular);
    double worst_middle = 0.0;
    double worst_outer  = 0.0;

    for(int sample = 0; sample < draws; ++sample)
    {
        const Eigen::Vector3d angles_radians = crowded.euler_triple_radians();

        worst_middle = std::max(worst_middle, nearest_of(angles_radians.y(), singular));
        worst_outer  = std::max(worst_outer, nearest_of(angles_radians.x(), singular));
    }

    REQUIRE(worst_middle <= crowding_extent);
    REQUIRE(worst_outer > crowding_extent);
}

TEST_CASE("a_near_singular_rotation_is_composed_from_an_angle_drawn_the_same_way")
{
    constexpr std::array<double, 2> singular{0.0, half_turn};
    case_source crowded = case_source::for_slot(recorded_seed, "rotation_member", spread::near_singular);
    double furthest     = 0.0;

    for(int sample = 0; sample < draws; ++sample)
    {
        const Eigen::AngleAxisd turned(crowded.rotation_member());

        furthest = std::max(furthest, nearest_of(turned.angle(), singular));
    }

    REQUIRE(furthest <= crowding_extent + default_tolerance);
}

TEST_CASE("a_near_singular_unit_twist_always_takes_the_branch_with_no_angular_part")
{
    case_source crowded = case_source::for_slot(recorded_seed, "unit_twist", spread::near_singular);
    bool every_silent   = true;

    for(int sample = 0; sample < draws; ++sample)
    {
        const Eigen::Vector<double, 6> axis = crowded.unit_twist();

        every_silent = every_silent && axis.head<3>() == Eigen::Vector3d::Zero() && std::fabs(axis.tail<3>().norm() - 1.0) <= default_tolerance;
    }

    REQUIRE(every_silent);
}

TEST_CASE("a_role_with_no_singular_neighbourhood_draws_from_the_same_distribution_under_both_spreads")
{
    case_source crowded   = case_source::for_slot(recorded_seed, "position_metres", spread::near_singular);
    Eigen::Vector3d total = Eigen::Vector3d::Zero();
    double furthest       = 0.0;

    for(int sample = 0; sample < draws; ++sample)
    {
        const Eigen::Vector3d at = crowded.position_metres();

        total += at;
        furthest = std::max(furthest, at.cwiseAbs().maxCoeff());
    }

    REQUIRE(furthest <= position_extent);
    REQUIRE(furthest > 0.9 * position_extent);
    REQUIRE((total / static_cast<double>(draws)).cwiseAbs().maxCoeff() < 4.0 * std::sqrt(position_extent * position_extent / 3.0 / static_cast<double>(draws)));
}
