#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/comparators.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <cmath>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

// Generic in its axis, so that no term of the membership test is satisfied by an axis lining up with
// the frame.
Eigen::Matrix3d generic_rotation()
{
    return Eigen::AngleAxisd(0.9, Eigen::Vector3d(0.3, -0.7, 0.5).normalized()).toRotationMatrix();
}

// Orthonormal still, and of determinant minus one.
Eigen::Matrix3d reflected(const Eigen::Matrix3d &r)
{
    Eigen::Matrix3d m(r);
    m.col(0) = -m.col(0);

    return m;
}

// A multiple of one column added to another, which leaves the determinant exactly where it was.
Eigen::Matrix3d sheared(const Eigen::Matrix3d &r, double by)
{
    Eigen::Matrix3d m(r);
    m.col(0) += by * r.col(1);

    return m;
}

bool differs_however_wide_the_bound(const residual &seen)
{
    return verdict_of(seen, tolerance_of(seen.kind)) == agreement::differed && verdict_of(seen, tolerance_pair{1.0e300, 1.0e300}) == agreement::differed;
}

}

// Each of the three converts through Eigen's axis-angle form to an angle of zero, so each one read as
// an exact match before the membership test was asked.
TEST_CASE("a_product_that_is_not_a_rotation_carries_no_angle_and_differs_however_wide_the_bound")
{
    const Eigen::Matrix3d held(generic_rotation());

    for(const Eigen::Matrix3d &against : {reflected(held), Eigen::Matrix3d(held * 2.0), sheared(held, 1.0e-6)})
    {
        const residual seen = geodesic_residual(held, against);

        REQUIRE(std::isinf(seen.magnitude));
        REQUIRE(differs_however_wide_the_bound(seen));
    }
}

// The reported kind is the slot's own whatever the input, since a run's worst residual is folded
// across cases in one kind and a report names that kind.
TEST_CASE("a_refused_membership_still_reports_the_kind_the_comparator_measures_in")
{
    const Eigen::Matrix3d held(generic_rotation());

    REQUIRE(geodesic_residual(held, reflected(held)).kind == residual_kind::geodesic);
    REQUIRE(geodesic_residual(held, sheared(held, 1.0e-6)).kind == residual_kind::geodesic);
    REQUIRE(geodesic_residual(held, held).kind == residual_kind::geodesic);
}

// Neither term alone admits only rotations: a reflection is exactly orthonormal and a shear leaves
// the determinant untouched, so each class is caught by the term the other class escapes.
TEST_CASE("the_two_terms_of_the_membership_test_each_catch_a_class_the_other_misses")
{
    const Eigen::Matrix3d turned(generic_rotation());
    const Eigen::Matrix3d flipped(reflected(turned));
    const Eigen::Matrix3d skewed(sheared(turned, 1.0e-6));

    REQUIRE((flipped.transpose() * flipped - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() < 1.0e-15);
    REQUIRE(std::fabs(flipped.determinant() + 1.0) < 1.0e-15);
    REQUIRE((skewed.transpose() * skewed - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 1.0e-7);
    REQUIRE(std::fabs(skewed.determinant() - 1.0) < 1.0e-15);
}

// The scaled class is the tightest of the three against the tree's own membership tolerance: a
// scaling by one part in e moves the orthonormality term by 2e and the determinant term by 3e.
TEST_CASE("the_membership_test_admits_a_scaling_well_below_the_tolerance_and_refuses_one_above_it")
{
    const Eigen::Matrix3d held(generic_rotation());

    REQUIRE(std::isfinite(geodesic_residual(held, Eigen::Matrix3d(held * (1.0 + 1.0e-13))).magnitude));
    REQUIRE(std::isinf(geodesic_residual(held, Eigen::Matrix3d(held * (1.0 + 1.0e-11))).magnitude));
}

TEST_CASE("a_pose_whose_rotation_block_is_not_a_rotation_carries_no_angle_and_keeps_its_measured_distance")
{
    Eigen::Matrix4d held(Eigen::Matrix4d::Identity());
    Eigen::Matrix4d against(Eigen::Matrix4d::Identity());
    held.block<3, 3>(0, 0)    = generic_rotation();
    against.block<3, 3>(0, 0) = generic_rotation() * 2.0;
    against.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.0, 0.25);

    const residual seen = pose_residual(held, against);

    REQUIRE(std::isinf(seen.magnitude));
    REQUIRE(seen.kind == residual_kind::pose);
    REQUIRE(std::fabs(seen.linear_error_metres - 0.25) < 1.0e-15);
    REQUIRE(differs_however_wide_the_bound(seen));
}

TEST_CASE("a_rotation_still_reads_its_own_angle_once_the_membership_test_is_asked")
{
    const Eigen::Matrix3d held(generic_rotation());
    const Eigen::Matrix3d turned(held * Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix());

    REQUIRE(geodesic_residual(held, held).magnitude == 0.0);
    REQUIRE(std::fabs(geodesic_residual(held, turned).magnitude - 0.7) < 1.0e-12);
}
