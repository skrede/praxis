#include "praxis/rigid_motion/baseline/frame.h"

#include <Eigen/Geometry>

#include <cartan/lie/axis_angle.h>

#include <array>
#include <cstdint>

namespace praxis::rigid_motion {

namespace {

using lie_axis_angle = cartan::axis_angle<double>;

rotation axis_rotation(std::uint8_t axis, double radians)
{
    return Eigen::AngleAxisd(radians, Eigen::Vector3d::Unit(axis)).toRotationMatrix();
}

}

// Eigen's extraction covers both the Tait-Bryan and the proper-Euler families; its first angle is
// confined to [0, pi], which is how it picks one of the two decompositions that describe the same
// rotation.
Eigen::Vector3d euler_from_rotation_matrix(const rotation &r, axis_order order)
{
    const std::array<std::uint8_t, 3> axes = axis_indices(order);
    return r.eulerAngles(axes[0], axes[1], axes[2]);
}

rotation rotate_x(double radians)
{
    return axis_rotation(0, radians);
}

rotation rotate_y(double radians)
{
    return axis_rotation(1, radians);
}

rotation rotate_z(double radians)
{
    return axis_rotation(2, radians);
}

rotation rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z)
{
    rotation r;
    r.col(0) = x;
    r.col(1) = y;
    r.col(2) = z;
    return r;
}

rotation rotation_matrix_from_euler(const Eigen::Vector3d &e, axis_order order)
{
    const std::array<std::uint8_t, 3> axes = axis_indices(order);
    return axis_rotation(axes[0], e[0]) * axis_rotation(axes[1], e[1]) * axis_rotation(axes[2], e[2]);
}

rotation rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double radians)
{
    return cartan::from_axis_angle(lie_axis_angle{axis, radians}).matrix();
}

rotation rotation_matrix_from_transform(const transform &tf)
{
    return tf.block<3, 3>(0, 0);
}

transform transformation_matrix_from_position(const Eigen::Vector3d &p)
{
    return transformation_matrix_from_rotation_position(rotation::Identity(), p);
}

transform transformation_matrix_from_rotation(const rotation &r)
{
    return transformation_matrix_from_rotation_position(r, Eigen::Vector3d::Zero());
}

transform transformation_matrix_from_rotation_position(const rotation &r, const Eigen::Vector3d &p)
{
    transform tf         = transform::Identity();
    tf.block<3, 3>(0, 0) = r;
    tf.block<3, 1>(0, 3) = p;
    return tf;
}

// Lynch & Park, Modern Robotics, Prop. 3.13: the inverse of (R, p) is (R^T, -R^T p), which is not
// the transpose of the 4x4 and agrees with a general matrix inverse only up to rounding.
transform inverse(const transform &tf)
{
    const rotation reversed = rotation_matrix_from_transform(tf).transpose();

    return transformation_matrix_from_rotation_position(reversed, -(reversed * tf.block<3, 1>(0, 3)));
}

}
