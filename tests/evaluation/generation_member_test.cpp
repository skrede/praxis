#include "praxis/evaluation/tolerance.h"
#include "praxis/evaluation/generation.h"

#include "praxis/rigid_motion/types.h"

#include "praxis/trajectory/baseline/path.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <algorithm>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr std::uint64_t recorded_seed = 0x5EEDu;
constexpr int draws                   = 10000;

double four_errors_of(double variance)
{
    return 4.0 * std::sqrt(variance / static_cast<double>(draws));
}

transform reference_endpoint()
{
    transform tf         = transform::Identity();
    tf.block<3, 3>(0, 0) = Eigen::AngleAxisd(0.9, Eigen::Vector3d(0.3, -0.7, 0.5).normalized()).toRotationMatrix();
    tf.block<3, 1>(0, 3) << 0.21, -1.4, 0.66;

    return tf;
}

}

TEST_CASE("a_drawn_orthonormal_triple_is_unit_norm_mutually_orthogonal_and_right_handed")
{
    case_source drawn       = case_source::for_slot(recorded_seed, "orthonormal_triple");
    double worst_norm       = 0.0;
    double worst_orthogonal = 0.0;
    double worst_handedness = 0.0;

    for(int sample = 0; sample < draws; ++sample)
    {
        const Eigen::Matrix3d axes = drawn.orthonormal_triple();

        worst_norm       = std::max(worst_norm, (axes.colwise().norm().array() - 1.0).abs().maxCoeff());
        worst_orthogonal = std::max({worst_orthogonal, std::fabs(axes.col(0).dot(axes.col(1))), std::fabs(axes.col(0).dot(axes.col(2))), std::fabs(axes.col(1).dot(axes.col(2)))});
        worst_handedness = std::max(worst_handedness, (axes.col(0).cross(axes.col(1)) - axes.col(2)).cwiseAbs().maxCoeff());
    }

    REQUIRE(worst_norm <= default_tolerance);
    REQUIRE(worst_orthogonal <= default_tolerance);
    REQUIRE(worst_handedness <= default_tolerance);
}

TEST_CASE("a_drawn_rotation_is_orthonormal_with_a_determinant_of_one")
{
    case_source drawn        = case_source::for_slot(recorded_seed, "rotation_member");
    double worst_orthonormal = 0.0;
    double worst_determinant = 0.0;

    for(int sample = 0; sample < draws; ++sample)
    {
        const Eigen::Matrix3d turned = drawn.rotation_member();

        worst_orthonormal = std::max(worst_orthonormal, (turned.transpose() * turned - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff());
        worst_determinant = std::max(worst_determinant, std::fabs(turned.determinant() - 1.0));
    }

    REQUIRE(worst_orthonormal <= default_tolerance);
    REQUIRE(worst_determinant <= default_tolerance);
}

// Membership is asked of the path that already refuses a non-member rather than of a predicate
// written a second time here: a rotation off SO(3), or a bottom row that is not affine, is answered
// as a refusal instead of as an interpolated pose.
TEST_CASE("a_drawn_transform_stands_over_an_affine_bottom_row_and_is_accepted_as_a_path_endpoint")
{
    case_source drawn         = case_source::for_slot(recorded_seed, "transform_member");
    const transform elsewhere = reference_endpoint();
    double worst_bottom       = 0.0;
    bool every_member         = true;

    for(int sample = 0; sample < draws; ++sample)
    {
        const transform pose = drawn.transform_member();

        worst_bottom = std::max(worst_bottom, (pose.row(3) - Eigen::RowVector4d::UnitW()).cwiseAbs().maxCoeff());
        every_member = every_member && trajectory::screw(pose, elsewhere, 0.5).has_value();
    }

    REQUIRE(worst_bottom == 0.0);
    REQUIRE(every_member);
}

TEST_CASE("a_drawn_skew_symmetric_matrix_added_to_its_own_transpose_is_exactly_the_zero_matrix")
{
    case_source drawn = case_source::for_slot(recorded_seed, "skew_symmetric_member");
    bool every_zero   = true;
    double furthest   = 0.0;

    for(int sample = 0; sample < draws; ++sample)
    {
        const Eigen::Matrix3d crossed = drawn.skew_symmetric_member();

        every_zero = every_zero && (crossed + crossed.transpose()) == Eigen::Matrix3d::Zero();
        furthest   = std::max(furthest, crossed.cwiseAbs().maxCoeff());
    }

    REQUIRE(every_zero);
    REQUIRE(furthest > 1.0);
}

TEST_CASE("a_drawn_unit_twist_is_unit_in_one_half_reaches_both_branches_and_is_never_entirely_zero")
{
    case_source drawn = case_source::for_slot(recorded_seed, "unit_twist");
    bool every_unit   = true;
    int silent        = 0;

    for(int sample = 0; sample < draws; ++sample)
    {
        const Eigen::Vector<double, 6> axis = drawn.unit_twist();
        const bool turning                  = std::fabs(axis.head<3>().norm() - 1.0) <= default_tolerance;
        const bool sliding                  = axis.head<3>() == Eigen::Vector3d::Zero() && std::fabs(axis.tail<3>().norm() - 1.0) <= default_tolerance;

        every_unit = every_unit && (turning || sliding);
        silent += sliding ? 1 : 0;
    }

    REQUIRE(every_unit);
    REQUIRE(std::fabs(static_cast<double>(silent) / static_cast<double>(draws) - 0.5) < four_errors_of(0.25));
}

TEST_CASE("a_drawn_twist_is_unconstrained_and_finite")
{
    case_source drawn = case_source::for_slot(recorded_seed, "twist_member");
    bool every_finite = true;
    double furthest   = 0.0;

    for(int sample = 0; sample < draws; ++sample)
    {
        const Eigen::Vector<double, 6> both = drawn.twist_member();

        every_finite = every_finite && both.allFinite();
        furthest     = std::max(furthest, both.cwiseAbs().maxCoeff());
    }

    REQUIRE(every_finite);
    REQUIRE(furthest > 1.0);
}
