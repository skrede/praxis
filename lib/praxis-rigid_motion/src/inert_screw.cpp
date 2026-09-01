#include "praxis/rigid_motion/screw.h"

namespace praxis::rigid_motion::inert {

matrix3 skew_symmetric(const Eigen::Vector3d &)
{
    return matrix3::Identity();
}

Eigen::Vector3d from_skew_symmetric(const matrix3 &)
{
    return Eigen::Vector3d::Zero();
}

expected<adjoint, refusal> adjoint_matrix_from_rotation_position(const rotation &, const Eigen::Vector3d &)
{
    return unexpected(refusal::not_implemented);
}

expected<adjoint, refusal> adjoint_matrix_from_transform(const transform &)
{
    return unexpected(refusal::not_implemented);
}

expected<twist, refusal> adjoint_map(const twist &, const transform &)
{
    return unexpected(refusal::not_implemented);
}

twist twist_from_angular_linear(const Eigen::Vector3d &, const Eigen::Vector3d &)
{
    return twist::Zero();
}

expected<twist, refusal> twist_from_screw(const Eigen::Vector3d &, const Eigen::Vector3d &, double, double)
{
    return unexpected(refusal::not_implemented);
}

matrix4 twist_matrix_from_angular_linear(const Eigen::Vector3d &, const Eigen::Vector3d &)
{
    return matrix4::Identity();
}

matrix4 twist_matrix_from_twist(const twist &)
{
    return matrix4::Identity();
}

screw_axis screw_axis_from_angular_linear(const Eigen::Vector3d &, const Eigen::Vector3d &)
{
    return screw_axis::Zero();
}

expected<screw_axis, refusal> screw_axis_from_point_direction_pitch(const Eigen::Vector3d &, const Eigen::Vector3d &, double)
{
    return unexpected(refusal::not_implemented);
}

rotation matrix_exponential_so3(const Eigen::Vector3d &, double)
{
    return rotation::Identity();
}

transform matrix_exponential_se3(const Eigen::Vector3d &, const Eigen::Vector3d &, double)
{
    return transform::Identity();
}

transform matrix_exponential_screw(const screw_axis &, double)
{
    return transform::Identity();
}

expected<std::pair<Eigen::Vector3d, double>, refusal> matrix_logarithm_so3(const rotation &)
{
    return unexpected(refusal::not_implemented);
}

expected<std::pair<screw_axis, double>, refusal> matrix_logarithm_se3_rp(const rotation &, const Eigen::Vector3d &)
{
    return unexpected(refusal::not_implemented);
}

expected<std::pair<screw_axis, double>, refusal> matrix_logarithm_se3(const transform &)
{
    return unexpected(refusal::not_implemented);
}

}
