#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/angles.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <random>
#include <numbers>
#include <algorithm>
#include <type_traits>

using namespace praxis;

static_assert(twist::RowsAtCompileTime == 6 && twist::ColsAtCompileTime == 1);
static_assert(screw_axis::RowsAtCompileTime == 6 && screw_axis::ColsAtCompileTime == 1);
static_assert(adjoint::RowsAtCompileTime == 6 && adjoint::ColsAtCompileTime == 6);
static_assert(transform::RowsAtCompileTime == 4 && transform::ColsAtCompileTime == 4);
static_assert(matrix4::RowsAtCompileTime == 4 && matrix4::ColsAtCompileTime == 4);
static_assert(rotation::RowsAtCompileTime == 3 && rotation::ColsAtCompileTime == 3);
static_assert(matrix3::RowsAtCompileTime == 3 && matrix3::ColsAtCompileTime == 3);

static_assert(std::is_same_v<twist::Scalar, double>);
static_assert(std::is_same_v<transform::Scalar, double>);

TEST_CASE("half_a_turn_in_degrees_converts_to_pi")
{
    REQUIRE(is_approx_equal(to_radians(180.0), std::numbers::pi_v<double>));
    REQUIRE(is_approx_equal(to_degrees(std::numbers::pi_v<double>), 180.0));
}

TEST_CASE("degrees_survive_a_conversion_round_trip")
{
    for(double degrees = -360.0; degrees <= 360.0; degrees += 7.5)
        REQUIRE(is_approx_equal(to_degrees(to_radians(degrees)), degrees));
}

// Derives the value default_tolerance carries: the worst round-trip error the module's own
// arithmetic produces, measured rather than asserted, with a decade of headroom demanded on top.
TEST_CASE("the_default_tolerance_stays_a_decade_above_the_worst_round_trip_error")
{
    std::mt19937_64 rng(0x5EEDu);
    std::normal_distribution<double> gauss(0.0, 1.0);
    std::uniform_real_distribution<double> angle_of(0.0, std::numbers::pi_v<double>);
    std::uniform_real_distribution<double> position_of(-2.0, 2.0);
    std::uniform_real_distribution<double> degrees_of(-360.0, 360.0);

    double worst = 0.0;
    for(int sample = 0; sample < 100000; ++sample)
    {
        Eigen::Vector3d axis(gauss(rng), gauss(rng), gauss(rng));
        while(axis.norm() < 1.0e-6)
            axis = Eigen::Vector3d(gauss(rng), gauss(rng), gauss(rng));
        const rotation r         = Eigen::AngleAxisd(angle_of(rng), axis.normalized()).toRotationMatrix();
        const rotation recovered = Eigen::AngleAxisd(r).toRotationMatrix();

        transform pose         = transform::Identity();
        pose.block<3, 3>(0, 0) = r;
        pose.block<3, 1>(0, 3) = Eigen::Vector3d(position_of(rng), position_of(rng), position_of(rng));

        const double degrees = degrees_of(rng);
        worst                = std::max({worst, (recovered - r).cwiseAbs().maxCoeff(), (pose * pose.inverse() - transform::Identity()).cwiseAbs().maxCoeff(),
                                         std::fabs(to_degrees(to_radians(degrees)) - degrees)});
    }

    REQUIRE(worst > 0.0);
    REQUIRE(worst * 10.0 < default_tolerance);
}
