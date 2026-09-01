#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/tolerance.h"
#include "praxis/evaluation/comparators.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <numbers>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr double a_full_turn = 2.0 * std::numbers::pi_v<double>;

Eigen::Vector<double, 6> axis_of(double wx, double wy, double wz, double vx, double vy, double vz)
{
    Eigen::Vector<double, 6> written;

    written << wx, wy, wz, vx, vy, vz;

    return written;
}

}

TEST_CASE("a_rotation_logarithm_compared_against_itself_is_exactly_zero")
{
    const Eigen::Vector3d axis(Eigen::Vector3d(1.0, 2.0, 3.0).normalized());
    const residual seen = log_up_to_branch_rotation_residual(axis, 0.7, axis, 0.7);

    REQUIRE(seen.kind == residual_kind::log_up_to_branch);
    REQUIRE(seen.magnitude == 0.0);
    REQUIRE(seen.linear_error_metres == 0.0);
}

TEST_CASE("a_rotation_logarithm_agrees_with_the_negated_axis_and_negated_angle_that_names_it")
{
    const Eigen::Vector3d axis(Eigen::Vector3d(1.0, -2.0, 0.5).normalized());

    REQUIRE(log_up_to_branch_rotation_residual(axis, 0.9, -axis, -0.9).magnitude == 0.0);
}

TEST_CASE("a_rotation_logarithm_agrees_with_the_same_angle_a_full_turn_further_on")
{
    const Eigen::Vector3d axis(Eigen::Vector3d(0.0, 1.0, 1.0).normalized());
    const residual seen = log_up_to_branch_rotation_residual(axis, 0.3, axis, 0.3 + a_full_turn);

    REQUIRE(seen.magnitude <= default_tolerance);
    REQUIRE(verdict_of(seen, tolerance_of(residual_kind::log_up_to_branch)) == agreement::agreed);
}

TEST_CASE("a_rotation_logarithm_naming_another_element_answers_the_angle_between_the_two")
{
    const Eigen::Vector3d axis(Eigen::Vector3d::UnitZ());
    const residual seen = log_up_to_branch_rotation_residual(axis, 0.2, axis, 0.9);

    REQUIRE(std::fabs(seen.magnitude - 0.7) < 1.0e-12);
    REQUIRE(verdict_of(seen, tolerance_of(residual_kind::log_up_to_branch)) == agreement::differed);
}

TEST_CASE("a_rotation_logarithm_of_no_rotation_at_all_answers_zero_whatever_the_axis_is")
{
    const Eigen::Vector3d collapsed(Eigen::Vector3d::Zero());
    const Eigen::Vector3d named(Eigen::Vector3d::UnitX());
    const residual seen = log_up_to_branch_rotation_residual(collapsed, 0.0, named, 0.0);

    REQUIRE(std::isfinite(seen.magnitude));
    REQUIRE(seen.magnitude == 0.0);
    REQUIRE(log_up_to_branch_rotation_residual(collapsed, 0.0, collapsed, 0.0).magnitude == 0.0);
    REQUIRE(log_up_to_branch_rotation_residual(named, 0.0, named, 1.4).magnitude > 0.0);
}

TEST_CASE("a_pose_logarithm_compared_against_itself_is_exactly_zero_in_both_halves")
{
    const Eigen::Vector<double, 6> axis(axis_of(0.0, 0.0, 1.0, 1.0, 2.0, 0.0));
    const residual seen = log_up_to_branch_pose_residual(axis, 0.6, axis, 0.6);

    REQUIRE(seen.kind == residual_kind::log_up_to_branch);
    REQUIRE(seen.magnitude == 0.0);
    REQUIRE(seen.linear_error_metres == 0.0);
}

TEST_CASE("a_pose_logarithm_agrees_with_the_negated_pair_and_with_a_full_turn_further_on")
{
    const Eigen::Vector<double, 6> axis(axis_of(0.0, 1.0, 0.0, 0.5, 0.0, -1.5));
    const residual negated = log_up_to_branch_pose_residual(axis, 1.1, -axis, -1.1);
    const residual around  = log_up_to_branch_pose_residual(axis, 1.1, axis, 1.1 + a_full_turn);

    REQUIRE(negated.magnitude == 0.0);
    REQUIRE(negated.linear_error_metres == 0.0);
    REQUIRE(around.magnitude <= default_tolerance);
    REQUIRE(around.linear_error_metres <= default_tolerance);
}

TEST_CASE("a_pose_logarithm_naming_another_element_answers_both_halves_of_the_pose_error")
{
    const Eigen::Vector<double, 6> axis(axis_of(0.0, 0.0, 1.0, 2.0, 0.0, 0.0));
    const residual seen = log_up_to_branch_pose_residual(axis, 0.2, axis, 0.5);

    REQUIRE(std::fabs(seen.magnitude - 0.3) < 1.0e-12);
    REQUIRE(seen.linear_error_metres > 0.0);
    REQUIRE(verdict_of(seen, tolerance_of(residual_kind::log_up_to_branch)) == agreement::differed);
}

TEST_CASE("a_pose_logarithm_whose_angular_half_is_zero_is_a_translation_rather_than_a_singularity")
{
    const Eigen::Vector<double, 6> along(axis_of(0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
    const residual same    = log_up_to_branch_pose_residual(along, 3.0, along, 3.0);
    const residual further = log_up_to_branch_pose_residual(along, 3.0, along, 5.0);

    REQUIRE(std::isfinite(same.magnitude));
    REQUIRE(std::isfinite(same.linear_error_metres));
    REQUIRE(same.magnitude == 0.0);
    REQUIRE(same.linear_error_metres == 0.0);
    REQUIRE(further.magnitude == 0.0);
    REQUIRE(std::fabs(further.linear_error_metres - 2.0) < 1.0e-12);
    REQUIRE(verdict_of(further, tolerance_of(residual_kind::log_up_to_branch)) == agreement::differed);
}

TEST_CASE("a_pose_logarithm_agrees_with_a_negated_pure_translation_that_names_the_same_element")
{
    const Eigen::Vector<double, 6> along(axis_of(0.0, 0.0, 0.0, 0.6, -0.8, 0.0));
    const residual seen = log_up_to_branch_pose_residual(along, 2.5, -along, -2.5);

    REQUIRE(seen.magnitude == 0.0);
    REQUIRE(seen.linear_error_metres == 0.0);
}
