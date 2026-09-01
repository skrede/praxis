#include "fixtures.h"

#include "praxis/rigid_motion.h"

#include "praxis/rigid_motion/angles.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>

using namespace praxis;
using namespace praxis::rigid_motion;
using namespace praxis::conformance;

namespace {

const frame_ops ops = baseline().frame;

constexpr std::array<axis_order, 12> orders{axis_order::xyz, axis_order::xzy, axis_order::yxz, axis_order::yzx, axis_order::zxy, axis_order::zyx,
                                            axis_order::xyx, axis_order::xzx, axis_order::yxy, axis_order::yzy, axis_order::zxz, axis_order::zyz};

}

TEST_CASE("euler_zyx_from_rotation_matrix_arbitrary")
{
    const Eigen::Vector3d e{to_radians(45.0), to_radians(60.0), to_radians(-22.5)};
    CHECK(ops.euler_from_rotation_matrix(intrinsic_zyx(e), axis_order::zyx).isApprox(e, default_tolerance));
}

// At a vertical second axis only the sum or difference of the outer two angles is determined, and
// the recorded expected triples satisfy exactly that relation. Asserting the relation rather than
// one arbitrary representative is what makes the case independent of which of the infinitely many
// equivalent triples an extraction happens to return.
TEST_CASE("euler_zyx_from_rotation_matrix_gimbal_p")
{
    const Eigen::Vector3d e{to_radians(45.0), to_radians(90.0), to_radians(-22.5)};
    const Eigen::Vector3d result = ops.euler_from_rotation_matrix(intrinsic_zyx(e), axis_order::zyx);

    CHECK(same_angle(result[1], to_radians(90.0)));
    CHECK(same_angle(result[0] - result[2], to_radians(67.5)));
    CHECK(ops.rotation_matrix_from_euler(result, axis_order::zyx).isApprox(intrinsic_zyx(e), default_tolerance));
}

TEST_CASE("euler_zyx_from_rotation_matrix_gimbal_n")
{
    const Eigen::Vector3d e{to_radians(45.0), to_radians(-90.0), to_radians(-22.5)};
    const Eigen::Vector3d result = ops.euler_from_rotation_matrix(intrinsic_zyx(e), axis_order::zyx);

    CHECK(same_angle(result[1], to_radians(-90.0)));
    CHECK(same_angle(result[0] + result[2], to_radians(22.5)));
    CHECK(ops.rotation_matrix_from_euler(result, axis_order::zyx).isApprox(intrinsic_zyx(e), default_tolerance));
}

TEST_CASE("every_axis_order_round_trips_a_rotation_through_its_euler_angles")
{
    const Eigen::Vector3d e{to_radians(37.0), to_radians(52.0), to_radians(-19.0)};

    for(axis_order order : orders)
    {
        const rotation r         = ops.rotation_matrix_from_euler(e, order);
        const Eigen::Vector3d ea = ops.euler_from_rotation_matrix(r, order);
        CHECK(ops.rotation_matrix_from_euler(ea, order).isApprox(r, default_tolerance));
    }
}

TEST_CASE("each_axis_order_builds_a_different_rotation")
{
    const Eigen::Vector3d e{to_radians(37.0), to_radians(52.0), to_radians(-19.0)};

    for(std::size_t i = 1; i < orders.size(); ++i)
        CHECK_FALSE(ops.rotation_matrix_from_euler(e, orders[i]).isApprox(ops.rotation_matrix_from_euler(e, orders[0]), default_tolerance));
}

TEST_CASE("rot_x")
{
    CHECK(ops.rotate_x(to_radians(45.0)).isApprox(axis_angle_rotation(Eigen::Vector3d::UnitX(), to_radians(45.0)), default_tolerance));
}

TEST_CASE("rot_y")
{
    CHECK(ops.rotate_y(to_radians(45.0)).isApprox(axis_angle_rotation(Eigen::Vector3d::UnitY(), to_radians(45.0)), default_tolerance));
}

TEST_CASE("rot_z")
{
    CHECK(ops.rotate_z(to_radians(45.0)).isApprox(axis_angle_rotation(Eigen::Vector3d::UnitZ(), to_radians(45.0)), default_tolerance));
}

TEST_CASE("rotation_matrix_from_frame_axes")
{
    const rotation r = intrinsic_zyx(Eigen::Vector3d{to_radians(-22.5), to_radians(33.45), to_radians(282.87)});
    CHECK(ops.rotation_matrix_from_frame_axes(r.col(0), r.col(1), r.col(2)).isApprox(r, default_tolerance));
}

TEST_CASE("rotation_matrix_from_euler_zyx")
{
    const Eigen::Vector3d e{to_radians(45.0), to_radians(60.0), to_radians(-22.5)};
    CHECK(ops.rotation_matrix_from_euler(e, axis_order::zyx).isApprox(intrinsic_zyx(e), default_tolerance));
}

TEST_CASE("rotation_matrix_from_axis_angle")
{
    CHECK(ops.rotation_matrix_from_axis_angle(direction, angle).isApprox(axis_angle_rotation(direction, angle), default_tolerance));
}

TEST_CASE("rotation_matrix_tf")
{
    const rotation r = axis_angle_rotation(direction, to_radians(67.0));
    CHECK(ops.rotation_matrix_from_transform(assembled(r, point)).isApprox(r, default_tolerance));
}

TEST_CASE("transformation_matrix_p")
{
    CHECK(is_approx_equal(ops.transformation_matrix_from_position(point), assembled(rotation::Identity(), point)));
}

TEST_CASE("transformation_matrix_r")
{
    const rotation r = axis_angle_rotation(direction, to_radians(67.0));
    CHECK(is_approx_equal(ops.transformation_matrix_from_rotation(r), assembled(r, Eigen::Vector3d::Zero())));
}

TEST_CASE("transformation_matrix_rp")
{
    const rotation r = axis_angle_rotation(direction, to_radians(67.0));
    CHECK(is_approx_equal(ops.transformation_matrix_from_rotation_position(r, point), assembled(r, point)));
}

// The defining property, from both sides. A transpose of the whole matrix satisfies neither, while
// inverting twice and comparing to the original would be satisfied by any involution including
// that one.
TEST_CASE("the_inverse_undoes_a_rigid_motion_composed_in_either_order")
{
    const transform tf       = assembled(reference_rotation(), reference_position());
    const transform reversed = ops.inverse(tf);
    const transform before   = reversed * tf;
    const transform after    = tf * reversed;

    CHECK_FALSE(is_approx_equal(reversed, tf));
    CHECK_FALSE(is_approx_equal(reversed, inert::inverse(tf)));
    CHECK(is_approx_equal(before, transform::Identity()));
    CHECK(is_approx_equal(after, transform::Identity()));
}

TEST_CASE("the_inverse_is_the_transposed_rotation_applied_to_the_negated_position")
{
    const rotation r        = reference_rotation();
    const Eigen::Vector3d p = reference_position();

    CHECK(is_approx_equal(ops.inverse(assembled(r, p)), assembled(r.transpose(), -(r.transpose() * p))));
}

// A pure translation is the case a transpose of the whole matrix gets wrong while still agreeing on
// the rotation block.
TEST_CASE("the_inverse_of_a_pure_translation_negates_the_translation")
{
    CHECK(is_approx_equal(ops.inverse(assembled(rotation::Identity(), point)), assembled(rotation::Identity(), -point)));
}
