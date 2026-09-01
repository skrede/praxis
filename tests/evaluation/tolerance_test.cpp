#include "praxis/evaluation/tolerance.h"

#include "praxis/rigid_motion/types.h"

#include "praxis/trajectory/baseline/path.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

using namespace praxis;

namespace {

rotation turned()
{
    return Eigen::AngleAxisd(0.9, Eigen::Vector3d(0.3, -0.7, 0.5).normalized()).toRotationMatrix();
}

rotation nudged(double amount)
{
    rotation r = turned();
    r(0, 1) += amount;

    return r;
}

transform framed(const rotation &r)
{
    transform tf         = transform::Identity();
    tf.block<3, 3>(0, 0) = r;
    tf.block<3, 1>(0, 3) << 0.21, -1.4, 0.66;

    return tf;
}

transform sheared(double amount)
{
    rotation r = turned();
    r.col(0) += amount * r.col(1);

    return framed(r);
}

transform reflected()
{
    rotation mirror = rotation::Identity();
    mirror(2, 2)    = -1.0;

    return framed(rotation(turned() * mirror));
}

// Both task-space paths refuse an endpoint outside the special Euclidean group, so a near miss offered
// as the start frame is answered as a refusal rather than as an interpolated pose.
bool offered_as_a_path_endpoint(const transform &tf)
{
    return trajectory::screw(tf, framed(turned()), 0.5).has_value();
}

refusal refusal_for(const transform &tf)
{
    return trajectory::screw(tf, framed(turned()), 0.5).error();
}

}

TEST_CASE("the_rotation_comparison_holds_a_rotation_against_itself")
{
    REQUIRE(is_approx_equal(turned(), turned()));
    REQUIRE(is_approx_equal(rotation::Identity(), rotation(rotation::Identity())));
}

TEST_CASE("the_rotation_comparison_admits_a_perturbation_under_the_tolerance_and_refuses_one_over_it")
{
    REQUIRE(is_approx_equal(turned(), nudged(0.5 * default_tolerance)));
    REQUIRE_FALSE(is_approx_equal(turned(), nudged(5.0 * default_tolerance)));
}

TEST_CASE("the_rotation_comparison_answers_the_same_left_unqualified_as_at_the_extension_tolerance")
{
    for(double amount : {0.0, 0.5 * default_tolerance, 5.0 * default_tolerance, 1.0e-6})
        REQUIRE(is_approx_equal(turned(), nudged(amount)) == is_approx_equal(turned(), nudged(amount), default_tolerance));
}

TEST_CASE("a_rotation_of_unit_determinant_over_an_affine_bottom_row_is_a_rigid_motion")
{
    REQUIRE(offered_as_a_path_endpoint(framed(turned())));
    REQUIRE(offered_as_a_path_endpoint(transform::Identity()));
}

TEST_CASE("a_rotation_scaled_just_off_unity_is_not_a_rigid_motion")
{
    const transform scaled = framed(rotation(1.000000001 * turned()));

    REQUIRE_FALSE(offered_as_a_path_endpoint(scaled));
    REQUIRE(refusal_for(scaled) == refusal::degenerate);
}

TEST_CASE("a_rotation_sheared_in_one_column_is_not_a_rigid_motion")
{
    REQUIRE_FALSE(offered_as_a_path_endpoint(sheared(1.0e-9)));
    REQUIRE(refusal_for(sheared(1.0e-9)) == refusal::degenerate);
}

// Adding a multiple of one column to another leaves the determinant exactly where it was, so the
// orthonormality comparison is the only one of the three that sees a shear.
TEST_CASE("a_shear_just_over_the_tolerance_is_not_a_rigid_motion")
{
    REQUIRE_FALSE(offered_as_a_path_endpoint(sheared(1.1 * default_tolerance)));
    REQUIRE(refusal_for(sheared(1.1 * default_tolerance)) == refusal::degenerate);
}

// A reflection is exactly orthonormal, so the determinant is the only one of the three that sees it.
TEST_CASE("a_reflection_is_not_a_rigid_motion")
{
    REQUIRE_FALSE(offered_as_a_path_endpoint(reflected()));
    REQUIRE(refusal_for(reflected()) == refusal::degenerate);
}

TEST_CASE("a_transform_whose_bottom_row_is_not_affine_is_not_a_rigid_motion")
{
    transform leaning = framed(turned());
    leaning(3, 1)     = 1.0e-9;

    REQUIRE_FALSE(offered_as_a_path_endpoint(leaning));
    REQUIRE(refusal_for(leaning) == refusal::degenerate);
}
