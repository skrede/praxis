#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/comparators.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <random>
#include <numbers>
#include <algorithm>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr double half_a_turn = std::numbers::pi_v<double>;

Eigen::Vector3d drawn_direction(std::mt19937_64 &rng, std::normal_distribution<double> &gauss)
{
    Eigen::Vector3d axis(gauss(rng), gauss(rng), gauss(rng));

    while(axis.norm() < 1.0e-6)
        axis = Eigen::Vector3d(gauss(rng), gauss(rng), gauss(rng));

    return axis.normalized();
}

Eigen::Vector<double, 6> axis_of(double wx, double wy, double wz, double vx, double vy, double vz)
{
    Eigen::Vector<double, 6> written;

    written << wx, wy, wz, vx, vy, vz;

    return written;
}

}

TEST_CASE("an_element_wise_residual_is_the_greatest_absolute_difference_whichever_side_comes_first")
{
    const Eigen::Matrix3d held(Eigen::Matrix3d::Identity());
    Eigen::Matrix3d bent(held);
    bent(2, 1) += 0.25;

    REQUIRE(element_wise_residual(held, held).kind == residual_kind::element_wise);
    REQUIRE(element_wise_residual(held, held).magnitude == 0.0);
    REQUIRE(element_wise_residual(held, bent).magnitude == 0.25);
    REQUIRE(element_wise_residual(bent, held).magnitude == 0.25);
    REQUIRE(element_wise_residual(held, held).linear_error_metres == 0.0);
}

TEST_CASE("one_element_wise_comparator_serves_every_shape_a_binding_answers_with")
{
    const Eigen::Vector<double, 6> six(axis_of(0.0, 0.0, 1.0, 0.0, 2.0, 0.0));
    const Eigen::Matrix4d wide(Eigen::Matrix4d::Identity());
    Eigen::Matrix<double, 6, 6> square(Eigen::Matrix<double, 6, 6>::Zero());
    square(5, 0) = -0.5;

    REQUIRE(element_wise_residual(six, -six).magnitude == 4.0);
    REQUIRE(element_wise_residual(wide, wide).magnitude == 0.0);
    REQUIRE(element_wise_residual(square, Eigen::Matrix<double, 6, 6>::Zero()).magnitude == 0.5);
}

TEST_CASE("a_pose_residual_carries_its_two_halves_apart_and_never_sums_them")
{
    const Eigen::Matrix4d held(Eigen::Matrix4d::Identity());
    Eigen::Matrix4d moved(held);
    moved.block<3, 1>(0, 3) = Eigen::Vector3d(3.0, 4.0, 12.0);

    REQUIRE(pose_residual(held, held).magnitude == 0.0);
    REQUIRE(pose_residual(held, held).linear_error_metres == 0.0);
    REQUIRE(pose_residual(held, moved).kind == residual_kind::pose);
    REQUIRE(pose_residual(held, moved).magnitude == 0.0);
    REQUIRE(pose_residual(held, moved).linear_error_metres == 13.0);
}

TEST_CASE("a_pose_differing_only_in_orientation_answers_the_geodesic_angle_and_no_linear_error")
{
    Eigen::Matrix4d turned(Eigen::Matrix4d::Identity());
    turned.block<3, 3>(0, 0) = Eigen::AngleAxisd(0.75, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    const residual apart = pose_residual(Eigen::Matrix4d::Identity(), turned);

    REQUIRE(apart.linear_error_metres == 0.0);
    REQUIRE(std::fabs(apart.magnitude - 0.75) < 1.0e-12);
}

TEST_CASE("an_axis_agrees_with_its_own_negation_and_with_nothing_else")
{
    const Eigen::Vector<double, 6> held(axis_of(0.0, 0.0, 1.0, 0.0, 2.0, 0.0));
    const Eigen::Vector<double, 6> other(axis_of(0.0, 1.0, 0.0, 0.0, 2.0, 0.0));
    const double nearest = std::min((held - other).norm(), (held + other).norm());

    REQUIRE(axis_up_to_sign_residual(held, held).magnitude == 0.0);
    REQUIRE(axis_up_to_sign_residual(held, -held).magnitude == 0.0);
    REQUIRE(axis_up_to_sign_residual(held, held).kind == residual_kind::axis_up_to_sign);
    REQUIRE(axis_up_to_sign_residual(held, other).magnitude == nearest);
    REQUIRE(axis_up_to_sign_residual(other, held).magnitude == nearest);
    REQUIRE(axis_up_to_sign_residual(held, other).magnitude > 0.0);
}

TEST_CASE("a_rotation_against_itself_is_exactly_zero_and_a_half_turn_away_is_a_half_turn")
{
    const Eigen::Matrix3d held(Eigen::AngleAxisd(0.4, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()).toRotationMatrix());
    const Eigen::Matrix3d opposed(held * Eigen::AngleAxisd(half_a_turn, Eigen::Vector3d(3.0, -1.0, 2.0).normalized()).toRotationMatrix());

    REQUIRE(geodesic_residual(held, held).magnitude == 0.0);
    REQUIRE(std::fabs(geodesic_residual(held, opposed).magnitude - half_a_turn) < 1.0e-7);
}

TEST_CASE("the_geodesic_comparator_answers_a_finite_angle_in_the_closed_interval_from_zero_to_a_half_turn")
{
    std::mt19937_64 rng(0x5EEDu);
    std::normal_distribution<double> gauss(0.0, 1.0);
    std::uniform_real_distribution<double> bulk(0.0, half_a_turn);
    std::uniform_real_distribution<double> crowded(0.0, 1.0e-15);
    double widest    = 0.0;
    double narrowest = half_a_turn;

    for(int sample = 0; sample < 10000; ++sample)
    {
        const Eigen::Matrix3d held(Eigen::AngleAxisd(bulk(rng), drawn_direction(rng, gauss)).toRotationMatrix());
        double separation = bulk(rng);
        if(sample % 3 == 1)
            separation = crowded(rng);
        else if(sample % 3 == 2)
            separation = half_a_turn - crowded(rng);

        const Eigen::Matrix3d against(held * Eigen::AngleAxisd(separation, drawn_direction(rng, gauss)).toRotationMatrix());
        const double seen = geodesic_residual(held, against).magnitude;

        REQUIRE(std::isfinite(seen));
        REQUIRE(seen >= 0.0);
        REQUIRE(seen <= half_a_turn);
        widest    = std::max(widest, seen);
        narrowest = std::min(narrowest, seen);
    }

    REQUIRE(widest > 3.0);
    REQUIRE(narrowest < 1.0e-12);
}

TEST_CASE("a_residual_exactly_at_the_tolerance_agrees_and_one_representable_step_above_it_differs")
{
    const tolerance_pair allowed = tolerance_of(residual_kind::geodesic);
    const double a_step_above    = std::nextafter(allowed.magnitude, 1.0);

    REQUIRE(verdict_of(residual{residual_kind::geodesic, allowed.magnitude, 0.0}, allowed) == agreement::agreed);
    REQUIRE(verdict_of(residual{residual_kind::geodesic, a_step_above, 0.0}, allowed) == agreement::differed);
}

TEST_CASE("a_pose_verdict_holds_each_half_to_its_own_tolerance_and_agrees_only_when_neither_exceeds")
{
    const tolerance_pair allowed = tolerance_of(residual_kind::pose);
    const double turned_further  = std::nextafter(allowed.magnitude, 1.0);
    const double moved_further   = std::nextafter(allowed.linear_metres, 1.0);

    REQUIRE(verdict_of(residual{residual_kind::pose, allowed.magnitude, allowed.linear_metres}, allowed) == agreement::agreed);
    REQUIRE(verdict_of(residual{residual_kind::pose, turned_further, 0.0}, allowed) == agreement::differed);
    REQUIRE(verdict_of(residual{residual_kind::pose, 0.0, moved_further}, allowed) == agreement::differed);
}
