#include "praxis/rigid_motion/screw.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <utility>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

matrix3 constant_skew(const Eigen::Vector3d &)
{
    return matrix3::Constant(4.0);
}

}

TEST_CASE("the_skew_and_twist_slots_default_to_identity_or_zero")
{
    screw_ops ops{};

    REQUIRE(ops.skew_symmetric != nullptr);
    REQUIRE(ops.from_skew_symmetric != nullptr);
    REQUIRE(ops.twist_from_angular_linear != nullptr);
    REQUIRE(ops.twist_matrix_from_angular_linear != nullptr);
    REQUIRE(ops.twist_matrix_from_twist != nullptr);
    REQUIRE(ops.screw_axis_from_angular_linear != nullptr);

    REQUIRE(ops.skew_symmetric(Eigen::Vector3d::UnitX()).isApprox(matrix3::Identity(), default_tolerance));
    REQUIRE(ops.from_skew_symmetric(matrix3::Identity()).isZero(default_tolerance));
    REQUIRE(ops.twist_from_angular_linear(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()).isZero(default_tolerance));
    REQUIRE(ops.twist_matrix_from_angular_linear(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()).isApprox(matrix4::Identity(), default_tolerance));
    REQUIRE(ops.twist_matrix_from_twist(twist::Zero()).isApprox(matrix4::Identity(), default_tolerance));
    REQUIRE(ops.screw_axis_from_angular_linear(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()).isZero(default_tolerance));
}

TEST_CASE("the_matrix_exponential_slots_default_to_the_identity")
{
    screw_ops ops{};

    REQUIRE(ops.matrix_exponential_so3 != nullptr);
    REQUIRE(ops.matrix_exponential_se3 != nullptr);
    REQUIRE(ops.matrix_exponential_screw != nullptr);

    REQUIRE(ops.matrix_exponential_so3(Eigen::Vector3d::UnitZ(), 1.5).isApprox(rotation::Identity(), default_tolerance));
    REQUIRE(ops.matrix_exponential_se3(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), 1.5).isApprox(transform::Identity(), default_tolerance));
    REQUIRE(ops.matrix_exponential_screw(screw_axis::Zero(), 1.5).isApprox(transform::Identity(), default_tolerance));
}

// An identity adjoint, the twist handed back unchanged and a zero screw axis are all values a bound
// slot could have produced, so an unbound slot answering with one of them cannot be told from a
// bound one.
TEST_CASE("the_adjoint_and_screw_slots_default_to_refusing_rather_than_to_a_value")
{
    screw_ops ops{};

    REQUIRE(ops.adjoint_matrix_from_rotation_position != nullptr);
    REQUIRE(ops.adjoint_matrix_from_transform != nullptr);
    REQUIRE(ops.adjoint_map != nullptr);
    REQUIRE(ops.twist_from_screw != nullptr);
    REQUIRE(ops.screw_axis_from_point_direction_pitch != nullptr);

    const expected<adjoint, refusal> from_rp  = ops.adjoint_matrix_from_rotation_position(rotation::Identity(), Eigen::Vector3d::UnitX());
    const expected<adjoint, refusal> from_tf  = ops.adjoint_matrix_from_transform(transform::Identity());
    const expected<twist, refusal> mapped     = ops.adjoint_map(twist::Zero(), transform::Identity());
    const expected<twist, refusal> from_screw = ops.twist_from_screw(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), 0.5, 1.0);
    const expected<screw_axis, refusal> axis  = ops.screw_axis_from_point_direction_pitch(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), 0.5);

    REQUIRE_FALSE(from_rp.has_value());
    REQUIRE_FALSE(from_tf.has_value());
    REQUIRE_FALSE(mapped.has_value());
    REQUIRE_FALSE(from_screw.has_value());
    REQUIRE_FALSE(axis.has_value());

    CHECK(from_rp.error() == refusal::not_implemented);
    CHECK(from_tf.error() == refusal::not_implemented);
    CHECK(mapped.error() == refusal::not_implemented);
    CHECK(from_screw.error() == refusal::not_implemented);
    CHECK(axis.error() == refusal::not_implemented);
}

TEST_CASE("the_matrix_logarithm_slots_default_to_refusing_rather_than_to_a_zero_axis_and_a_zero_angle")
{
    screw_ops ops{};

    REQUIRE(ops.matrix_logarithm_so3 != nullptr);
    REQUIRE(ops.matrix_logarithm_se3_rp != nullptr);
    REQUIRE(ops.matrix_logarithm_se3 != nullptr);

    const expected<std::pair<Eigen::Vector3d, double>, refusal> so3 = ops.matrix_logarithm_so3(rotation::Identity());
    const expected<std::pair<screw_axis, double>, refusal> se3_rp   = ops.matrix_logarithm_se3_rp(rotation::Identity(), Eigen::Vector3d::UnitX());
    const expected<std::pair<screw_axis, double>, refusal> se3      = ops.matrix_logarithm_se3(transform::Identity());

    REQUIRE_FALSE(so3.has_value());
    REQUIRE_FALSE(se3_rp.has_value());
    REQUIRE_FALSE(se3.has_value());

    CHECK(so3.error() == refusal::not_implemented);
    CHECK(se3_rp.error() == refusal::not_implemented);
    CHECK(se3.error() == refusal::not_implemented);
}

TEST_CASE("assigning_one_slot_leaves_every_other_slot_on_its_default")
{
    screw_ops ops{.skew_symmetric = &constant_skew};

    REQUIRE(ops.skew_symmetric == &constant_skew);
    REQUIRE(ops.skew_symmetric(Eigen::Vector3d::UnitZ()).isApprox(matrix3::Constant(4.0), default_tolerance));

    REQUIRE(ops.from_skew_symmetric == &inert::from_skew_symmetric);
    REQUIRE_FALSE(ops.adjoint_map(twist::Zero(), transform::Identity()).has_value());
    REQUIRE(ops.matrix_exponential_screw(screw_axis::Zero(), 1.5).isApprox(transform::Identity(), default_tolerance));
}
