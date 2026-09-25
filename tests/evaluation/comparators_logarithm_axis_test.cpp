#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/tolerance.h"
#include "praxis/evaluation/comparators.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <cmath>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr double axis_unitness_bound = 1.0e-1;

Eigen::Vector<double, 6> axis_with_angular_half(const Eigen::Vector3d &w)
{
    Eigen::Vector<double, 6> written;

    written << w, Eigen::Vector3d(0.4, 0.0, -0.2);

    return written;
}

bool differs_however_wide_the_bound(const residual &seen)
{
    return verdict_of(seen, tolerance_of(seen.kind)) == agreement::differed && verdict_of(seen, tolerance_pair{1.0e300, 1.0e300}) == agreement::differed;
}

}

TEST_CASE("a_rotation_logarithm_whose_axis_is_twice_unit_length_names_no_element")
{
    const Eigen::Vector3d named(Eigen::Vector3d(1.0, 2.0, 3.0).normalized());
    const residual seen = log_up_to_branch_rotation_residual(2.0 * named, 0.7, named, 0.7);

    REQUIRE(seen.kind == residual_kind::log_up_to_branch);
    REQUIRE(!std::isfinite(seen.magnitude));
    REQUIRE(seen.linear_error_metres == 0.0);
    REQUIRE(differs_however_wide_the_bound(seen));
}

TEST_CASE("a_pose_logarithm_whose_angular_half_is_twice_unit_length_names_no_element_in_either_half")
{
    const Eigen::Vector3d named(Eigen::Vector3d(1.0, 2.0, 3.0).normalized());
    const residual seen = log_up_to_branch_pose_residual(axis_with_angular_half(2.0 * named), 0.7, axis_with_angular_half(named), 0.7);

    REQUIRE(seen.kind == residual_kind::log_up_to_branch);
    REQUIRE(!std::isfinite(seen.magnitude));
    REQUIRE(!std::isfinite(seen.linear_error_metres));
    REQUIRE(differs_however_wide_the_bound(seen));
}

TEST_CASE("an_axis_within_the_unitness_bound_names_the_direction_it_points_along")
{
    const Eigen::Vector3d named(Eigen::Vector3d(1.0, -2.0, 0.5).normalized());
    const Eigen::Vector3d nearly((1.0 + 0.5 * axis_unitness_bound) * named);
    const residual turned = log_up_to_branch_rotation_residual(nearly, 1.3, named, 1.3);
    const residual moved  = log_up_to_branch_pose_residual(axis_with_angular_half(nearly), 1.3, axis_with_angular_half(named), 1.3);

    REQUIRE(verdict_of(turned, tolerance_of(turned.kind)) == agreement::agreed);
    REQUIRE(verdict_of(moved, tolerance_of(moved.kind)) == agreement::agreed);
}

TEST_CASE("an_axis_beyond_the_unitness_bound_names_no_element_on_either_form")
{
    const Eigen::Vector3d named(Eigen::Vector3d(1.0, -2.0, 0.5).normalized());
    const Eigen::Vector3d beyond((1.0 + 10.0 * axis_unitness_bound) * named);
    const residual turned = log_up_to_branch_rotation_residual(beyond, 1.3, named, 1.3);
    const residual moved  = log_up_to_branch_pose_residual(axis_with_angular_half(beyond), 1.3, axis_with_angular_half(named), 1.3);

    REQUIRE(!std::isfinite(turned.magnitude));
    REQUIRE(!std::isfinite(moved.magnitude));
    REQUIRE(differs_however_wide_the_bound(turned));
    REQUIRE(differs_however_wide_the_bound(moved));
}

TEST_CASE("an_angular_part_at_the_collapse_boundary_is_neither_a_translation_nor_a_direction")
{
    const Eigen::Vector3d borderline(default_tolerance * Eigen::Vector3d::UnitX());
    const Eigen::Vector3d named(Eigen::Vector3d::UnitX());

    REQUIRE(!std::isfinite(log_up_to_branch_rotation_residual(borderline, 0.8, named, 0.8).magnitude));
    REQUIRE(!std::isfinite(log_up_to_branch_pose_residual(axis_with_angular_half(borderline), 0.8, axis_with_angular_half(named), 0.8).magnitude));
}

TEST_CASE("a_refusal_reads_the_same_whichever_side_the_axis_it_refuses_is_on")
{
    const Eigen::Vector3d named(Eigen::Vector3d::UnitY());
    const Eigen::Vector3d doubled(2.0 * named);
    const residual turned       = log_up_to_branch_rotation_residual(doubled, 0.5, named, 0.5);
    const residual swapped_turn = log_up_to_branch_rotation_residual(named, 0.5, doubled, 0.5);
    const residual moved        = log_up_to_branch_pose_residual(axis_with_angular_half(doubled), 0.5, axis_with_angular_half(named), 0.5);
    const residual swapped_move = log_up_to_branch_pose_residual(axis_with_angular_half(named), 0.5, axis_with_angular_half(doubled), 0.5);

    REQUIRE(turned.magnitude == swapped_turn.magnitude);
    REQUIRE(turned.linear_error_metres == swapped_turn.linear_error_metres);
    REQUIRE(moved.magnitude == swapped_move.magnitude);
    REQUIRE(moved.linear_error_metres == swapped_move.linear_error_metres);
}
