#include "fixtures.h"

#include "answered.h"

#include "praxis/rigid_motion.h"

#include "praxis/rigid_motion/angles.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <utility>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::rigid_motion;
using namespace praxis::conformance;

namespace {

const screw_ops ops = baseline().screw;

const Eigen::Vector3d probe{-0.7, 2.1, 0.4};

// A rotation scaled off SO(3): near enough to be handed by mistake, and far enough that the operations
// that read one have no answer for it.
rotation stretched_rotation()
{
    return 1.5 * reference_rotation();
}

transform stretched_pose()
{
    transform tf = assembled(reference_rotation(), reference_position());
    tf.block<3, 3>(0, 0) *= 1.5;

    return tf;
}

}

TEST_CASE("skew_symmetric")
{
    const matrix3 s = ops.skew_symmetric(point);

    CHECK(s.isApprox(cross_operator(point), default_tolerance));
    CHECK((s * probe).isApprox(point.cross(probe), default_tolerance));
    CHECK((s + s.transpose()).isZero(default_tolerance));
}

TEST_CASE("from_skew_symmetric")
{
    CHECK(ops.from_skew_symmetric(cross_operator(point)).isApprox(point, default_tolerance));
}

TEST_CASE("adjoint_matrix_rp")
{
    CHECK(answered(ops.adjoint_matrix_from_rotation_position(reference_rotation(), reference_position())).isApprox(reference_adjoint(), default_tolerance));
}

TEST_CASE("adjoint_matrix_tf")
{
    CHECK(answered(ops.adjoint_matrix_from_transform(assembled(reference_rotation(), reference_position()))).isApprox(reference_adjoint(), default_tolerance));
}

TEST_CASE("adjoint_map")
{
    CHECK(answered(ops.adjoint_map(adjoint_probe(), assembled(reference_rotation(), reference_position()))).isApprox(adjoint_probe_image(), default_tolerance));
}

// The adjoint is defined by the conjugation it performs on a twist matrix, so a wrong block layout
// or a transposed rotation fails here even where it happens to reproduce a recorded number.
TEST_CASE("the_adjoint_conjugates_the_twist_matrix_of_the_twist_it_maps")
{
    const transform tf   = assembled(reference_rotation(), reference_position());
    const twist mapped   = answered(ops.adjoint_map(adjoint_probe(), tf));
    const matrix4 direct = tf * ops.twist_matrix_from_twist(adjoint_probe()) * tf.inverse();

    CHECK(ops.twist_matrix_from_twist(mapped).isApprox(direct, default_tolerance));
    CHECK((answered(ops.adjoint_matrix_from_transform(tf)) * adjoint_probe()).isApprox(mapped, default_tolerance));
}

TEST_CASE("the_adjoint_of_a_composition_is_the_composition_of_the_adjoints")
{
    const transform first  = assembled(reference_rotation(), reference_position());
    const transform second = assembled(axis_angle_rotation(Eigen::Vector3d::UnitY(), 0.37), point);

    CHECK(answered(ops.adjoint_matrix_from_transform(first * second))
                  .isApprox(answered(ops.adjoint_matrix_from_transform(first)) * answered(ops.adjoint_matrix_from_transform(second)), default_tolerance));
}

TEST_CASE("screw_axis_wv_trans")
{
    const Eigen::Vector3d v = reference_screw().tail<3>().normalized();
    screw_axis expected;
    expected << Eigen::Vector3d::Zero(), v;

    CHECK(ops.screw_axis_from_angular_linear(Eigen::Vector3d::Zero(), v).isApprox(expected, default_tolerance));
}

TEST_CASE("screw_axis_wv_arbitrary")
{
    const screw_axis reference = reference_screw();
    CHECK(ops.screw_axis_from_angular_linear(reference.head<3>(), reference.tail<3>()).isApprox(reference, default_tolerance));
}

TEST_CASE("screw_axis_qsh")
{
    CHECK(answered(ops.screw_axis_from_point_direction_pitch(point, direction, pitch)).isApprox(reference_screw(), default_tolerance));
}

TEST_CASE("twist_wv")
{
    const twist reference = reference_twist();
    CHECK(ops.twist_from_angular_linear(reference.head<3>(), reference.tail<3>()).isApprox(reference, default_tolerance));
}

TEST_CASE("twist_qshv")
{
    CHECK(answered(ops.twist_from_screw(point, direction, pitch, angle)).isApprox(reference_twist(), default_tolerance));
}

TEST_CASE("twist_matrix_wv")
{
    const twist reference      = reference_twist();
    matrix4 expected           = matrix4::Zero();
    expected.block<3, 3>(0, 0) = cross_operator(reference.head<3>());
    expected.block<3, 1>(0, 3) = reference.tail<3>();

    CHECK(ops.twist_matrix_from_angular_linear(reference.head<3>(), reference.tail<3>()).isApprox(expected, default_tolerance));
}

TEST_CASE("twist_matrix_s")
{
    const twist reference = reference_twist();
    CHECK(ops.twist_matrix_from_twist(reference).isApprox(ops.twist_matrix_from_angular_linear(reference.head<3>(), reference.tail<3>()), default_tolerance));
}

TEST_CASE("exponential_logarithm")
{
    const transform pose                       = assembled(reference_rotation(), reference_position());
    const std::pair<screw_axis, double> logged = answered(ops.matrix_logarithm_se3(pose));

    CHECK(is_approx_equal(ops.matrix_exponential_screw(logged.first, logged.second), pose));
}

TEST_CASE("matrix_logarithm_r_arbitrary")
{
    const std::pair<Eigen::Vector3d, double> logged = answered(ops.matrix_logarithm_so3(reference_rotation()));

    CHECK(is_approx_equal(logged.second, angle));
    CHECK(logged.first.isApprox(direction, default_tolerance));
}

TEST_CASE("matrix_logarithm_r_rotx")
{
    const std::pair<Eigen::Vector3d, double> logged = answered(ops.matrix_logarithm_so3(axis_angle_rotation(Eigen::Vector3d::UnitX(), to_radians(180.0))));

    CHECK(is_approx_equal(logged.second, to_radians(180.0)));
    CHECK(logged.first.isApprox(Eigen::Vector3d::UnitX(), default_tolerance));
}

TEST_CASE("matrix_logarithm_r_roty")
{
    const std::pair<Eigen::Vector3d, double> logged = answered(ops.matrix_logarithm_so3(axis_angle_rotation(Eigen::Vector3d::UnitY(), to_radians(180.0))));

    CHECK(is_approx_equal(logged.second, to_radians(180.0)));
    CHECK(logged.first.isApprox(Eigen::Vector3d::UnitY(), default_tolerance));
}

TEST_CASE("matrix_logarithm_r_rotz")
{
    const std::pair<Eigen::Vector3d, double> logged = answered(ops.matrix_logarithm_so3(axis_angle_rotation(Eigen::Vector3d::UnitZ(), to_radians(180.0))));

    CHECK(is_approx_equal(logged.second, to_radians(180.0)));
    CHECK(logged.first.isApprox(Eigen::Vector3d::UnitZ(), default_tolerance));
}

TEST_CASE("matrix_logarithm_rp_ident")
{
    const std::pair<screw_axis, double> logged = answered(ops.matrix_logarithm_se3_rp(rotation::Identity(), point));

    CHECK(logged.first.head<3>().isZero(default_tolerance));
    CHECK(is_approx_equal(logged.second, point.norm()));
    CHECK(logged.first.tail<3>().isApprox(point.normalized(), default_tolerance));
}

TEST_CASE("matrix_logarithm_rp_arbitrary")
{
    const std::pair<screw_axis, double> logged = answered(ops.matrix_logarithm_se3_rp(reference_rotation(), reference_position()));

    CHECK(is_approx_equal(logged.second, angle));
    CHECK(logged.first.isApprox(reference_screw(), default_tolerance));
}

TEST_CASE("matrix_logarithm_t_ident")
{
    const std::pair<screw_axis, double> logged = answered(ops.matrix_logarithm_se3(assembled(rotation::Identity(), point)));

    CHECK(logged.first.head<3>().isZero(default_tolerance));
    CHECK(is_approx_equal(logged.second, point.norm()));
    CHECK(logged.first.tail<3>().isApprox(point.normalized(), default_tolerance));
}

TEST_CASE("matrix_logarithm_t_arbitrary")
{
    const std::pair<screw_axis, double> logged = answered(ops.matrix_logarithm_se3(assembled(reference_rotation(), reference_position())));

    CHECK(is_approx_equal(logged.second, angle));
    CHECK(logged.first.isApprox(reference_screw(), default_tolerance));
}

TEST_CASE("matrix_exponential_twt")
{
    CHECK(is_approx_equal(ops.matrix_exponential_screw(reference_screw(), angle), assembled(reference_rotation(), reference_position())));
}

TEST_CASE("matrix_exponential_wvt")
{
    const screw_axis reference = reference_screw();
    CHECK(is_approx_equal(ops.matrix_exponential_se3(reference.head<3>(), reference.tail<3>(), angle), assembled(reference_rotation(), reference_position())));
}

TEST_CASE("matrix_exponential_st")
{
    CHECK(ops.matrix_exponential_so3(direction, angle).isApprox(reference_rotation(), default_tolerance));
}

// A rotation about a point off the axis of a pitched screw is the geometric definition of the
// motion the exponential computes: Chasles' theorem, Lynch & Park, Modern Robotics, Ch. 3.3.2.
TEST_CASE("the_screw_exponential_agrees_with_the_geometric_screw_displacement")
{
    const rotation r                = axis_angle_rotation(direction, angle);
    const Eigen::Vector3d displaced = pitch * angle * direction + point - r * point;

    CHECK(is_approx_equal(ops.matrix_exponential_screw(reference_screw(), angle), assembled(r, displaced)));
}

TEST_CASE("the_rotation_exponential_and_logarithm_invert_each_other")
{
    const rotation r                                = ops.matrix_exponential_so3(direction, angle);
    const std::pair<Eigen::Vector3d, double> logged = answered(ops.matrix_logarithm_so3(r));

    CHECK(ops.matrix_exponential_so3(logged.first, logged.second).isApprox(r, default_tolerance));
}

// The adjoint map's answer on a transform it cannot read used to be the twist it was handed. That is
// the correct answer at the identity transform and a wrong one everywhere else, so nothing anywhere
// downstream could tell the failure from a success.
TEST_CASE("a_transform_that_is_not_a_rigid_motion_has_no_adjoint_to_carry_a_twist_through")
{
    const transform rigid                 = assembled(reference_rotation(), reference_position());
    const expected<twist, refusal> mapped = ops.adjoint_map(adjoint_probe(), stretched_pose());

    REQUIRE_FALSE(mapped.has_value());
    CHECK(mapped.error() == refusal::degenerate);

    // The rigid motion the stretched one was made from carries the probe somewhere else entirely, so
    // handing the probe back unchanged is a wrong answer here and not merely an unexamined one.
    CHECK_FALSE(adjoint_probe_image().isApprox(adjoint_probe(), default_tolerance));
    CHECK(answered(ops.adjoint_map(adjoint_probe(), rigid)).isApprox(adjoint_probe_image(), default_tolerance));
}

TEST_CASE("a_matrix_outside_the_group_it_is_read_from_is_refused_by_every_operation_that_reads_one")
{
    const expected<adjoint, refusal> from_rp                        = ops.adjoint_matrix_from_rotation_position(stretched_rotation(), reference_position());
    const expected<adjoint, refusal> from_tf                        = ops.adjoint_matrix_from_transform(stretched_pose());
    const expected<std::pair<Eigen::Vector3d, double>, refusal> so3 = ops.matrix_logarithm_so3(stretched_rotation());
    const expected<std::pair<screw_axis, double>, refusal> se3_rp   = ops.matrix_logarithm_se3_rp(stretched_rotation(), reference_position());
    const expected<std::pair<screw_axis, double>, refusal> se3      = ops.matrix_logarithm_se3(stretched_pose());

    REQUIRE_FALSE(from_rp.has_value());
    REQUIRE_FALSE(from_tf.has_value());
    REQUIRE_FALSE(so3.has_value());
    REQUIRE_FALSE(se3_rp.has_value());
    REQUIRE_FALSE(se3.has_value());

    CHECK(from_rp.error() == refusal::degenerate);
    CHECK(from_tf.error() == refusal::degenerate);
    CHECK(so3.error() == refusal::degenerate);
    CHECK(se3_rp.error() == refusal::degenerate);
    CHECK(se3.error() == refusal::degenerate);
}

TEST_CASE("a_twist_about_a_direction_that_names_no_axis_carries_the_screw_constructions_own_refusal")
{
    const expected<screw_axis, refusal> axis  = ops.screw_axis_from_point_direction_pitch(point, Eigen::Vector3d::Zero(), pitch);
    const expected<twist, refusal> from_screw = ops.twist_from_screw(point, Eigen::Vector3d::Zero(), pitch, angle);

    REQUIRE_FALSE(axis.has_value());
    REQUIRE_FALSE(from_screw.has_value());

    CHECK(axis.error() == refusal::degenerate);
    CHECK(from_screw.error() == axis.error());
}
