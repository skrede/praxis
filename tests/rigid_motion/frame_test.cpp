#include "praxis/rigid_motion/frame.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

rotation constant_rotation(double)
{
    return rotation::Constant(4.0);
}

transform displaced_turn()
{
    transform tf         = transform::Identity();
    tf.block<3, 3>(0, 0) = Eigen::AngleAxisd(1.1, Eigen::Vector3d(1.0, -2.0, 2.0).normalized()).toRotationMatrix();
    tf.block<3, 1>(0, 3) = Eigen::Vector3d(0.4, -1.3, 2.2);

    return tf;
}

}

TEST_CASE("the_euler_extraction_slot_defaults_to_zero")
{
    frame_ops ops{};

    REQUIRE(ops.euler_from_rotation_matrix != nullptr);
    REQUIRE(ops.euler_from_rotation_matrix(rotation::Identity(), axis_order::zyx).isZero(default_tolerance));
}

TEST_CASE("the_rotation_producing_slots_default_to_the_identity")
{
    frame_ops ops{};

    REQUIRE(ops.rotate_x != nullptr);
    REQUIRE(ops.rotate_y != nullptr);
    REQUIRE(ops.rotate_z != nullptr);
    REQUIRE(ops.rotation_matrix_from_frame_axes != nullptr);
    REQUIRE(ops.rotation_matrix_from_euler != nullptr);
    REQUIRE(ops.rotation_matrix_from_axis_angle != nullptr);
    REQUIRE(ops.rotation_matrix_from_transform != nullptr);

    REQUIRE(ops.rotate_x(1.5).isApprox(rotation::Identity(), default_tolerance));
    REQUIRE(ops.rotate_y(1.5).isApprox(rotation::Identity(), default_tolerance));
    REQUIRE(ops.rotate_z(1.5).isApprox(rotation::Identity(), default_tolerance));
    REQUIRE(ops.rotation_matrix_from_frame_axes(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()).isApprox(rotation::Identity(), default_tolerance));
    REQUIRE(ops.rotation_matrix_from_euler(Eigen::Vector3d::UnitX(), axis_order::xyz).isApprox(rotation::Identity(), default_tolerance));
    REQUIRE(ops.rotation_matrix_from_axis_angle(Eigen::Vector3d::UnitZ(), 1.5).isApprox(rotation::Identity(), default_tolerance));
    REQUIRE(ops.rotation_matrix_from_transform(transform::Identity()).isApprox(rotation::Identity(), default_tolerance));
}

TEST_CASE("the_transformation_producing_slots_default_to_the_identity")
{
    frame_ops ops{};

    REQUIRE(ops.transformation_matrix_from_position != nullptr);
    REQUIRE(ops.transformation_matrix_from_rotation != nullptr);
    REQUIRE(ops.transformation_matrix_from_rotation_position != nullptr);
    REQUIRE(ops.inverse != nullptr);

    REQUIRE(is_approx_equal(ops.transformation_matrix_from_position(Eigen::Vector3d::UnitX()), transform::Identity()));
    REQUIRE(is_approx_equal(ops.transformation_matrix_from_rotation(rotation::Identity()), transform::Identity()));
    REQUIRE(is_approx_equal(ops.transformation_matrix_from_rotation_position(rotation::Identity(), Eigen::Vector3d::UnitX()), transform::Identity()));
    REQUIRE(is_approx_equal(ops.inverse(displaced_turn()), transform::Identity()));
}

// The argument carries a substantial rotation and a substantial translation, so the identity the
// inert slot returns is distinguishable from an inverse rather than accidentally equal to one.
TEST_CASE("the_inert_inverse_does_not_undo_the_motion_it_is_given")
{
    frame_ops ops{};
    const transform undone = ops.inverse(displaced_turn()) * displaced_turn();

    REQUIRE_FALSE(is_approx_equal(undone, transform::Identity()));
}

TEST_CASE("assigning_one_slot_leaves_every_other_slot_on_its_default")
{
    frame_ops ops{.rotate_z = &constant_rotation};

    REQUIRE(ops.rotate_z == &constant_rotation);
    REQUIRE(ops.rotate_z(1.5).isApprox(rotation::Constant(4.0), default_tolerance));

    REQUIRE(ops.euler_from_rotation_matrix == &inert::euler_from_rotation_matrix);
    REQUIRE(ops.rotate_x == &inert::rotate_x);
    REQUIRE(ops.rotation_matrix_from_euler(Eigen::Vector3d::UnitX(), axis_order::xyz).isApprox(rotation::Identity(), default_tolerance));
    REQUIRE(is_approx_equal(ops.transformation_matrix_from_rotation_position(rotation::Identity(), Eigen::Vector3d::UnitX()), transform::Identity()));
}
