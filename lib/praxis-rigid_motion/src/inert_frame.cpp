#include "praxis/rigid_motion/frame.h"

namespace praxis::rigid_motion::inert {

Eigen::Vector3d euler_from_rotation_matrix(const rotation &, axis_order)
{
    return Eigen::Vector3d::Zero();
}

rotation rotate_x(double)
{
    return rotation::Identity();
}

rotation rotate_y(double)
{
    return rotation::Identity();
}

rotation rotate_z(double)
{
    return rotation::Identity();
}

rotation rotation_matrix_from_frame_axes(const Eigen::Vector3d &, const Eigen::Vector3d &, const Eigen::Vector3d &)
{
    return rotation::Identity();
}

rotation rotation_matrix_from_euler(const Eigen::Vector3d &, axis_order)
{
    return rotation::Identity();
}

rotation rotation_matrix_from_axis_angle(const Eigen::Vector3d &, double)
{
    return rotation::Identity();
}

rotation rotation_matrix_from_transform(const transform &)
{
    return rotation::Identity();
}

transform transformation_matrix_from_position(const Eigen::Vector3d &)
{
    return transform::Identity();
}

transform transformation_matrix_from_rotation(const rotation &)
{
    return transform::Identity();
}

transform transformation_matrix_from_rotation_position(const rotation &, const Eigen::Vector3d &)
{
    return transform::Identity();
}

transform inverse(const transform &)
{
    return transform::Identity();
}

}
