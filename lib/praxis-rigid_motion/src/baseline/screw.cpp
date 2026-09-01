#include "praxis/rigid_motion/baseline/screw.h"

#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/lie/twist.h>
#include <cartan/lie/hat_vee.h>
#include <cartan/lie/axis_angle.h>

namespace praxis::rigid_motion {

namespace {

using lie_rotation  = cartan::so3<double>;
using lie_transform = cartan::se3<double>;
using lie_twist     = cartan::twist<double>;
using lie_screw     = cartan::screw_motion<double>;

// Normalizing a twist to a unit screw axis and the magnitude that scales it is the same operation
// in both directions, and the pure-translation case is the one that has to be told apart: there the
// magnitude is a distance rather than an angle. Lynch & Park, Modern Robotics, Def. 3.24.
std::pair<screw_axis, double> unit_screw(const lie_twist &tw)
{
    const lie_screw motion = cartan::to_screw_motion(tw);
    const bool rotational  = motion.theta > 0.0;
    const lie_twist unit   = cartan::from_screw_motion(lie_screw{motion.axis, rotational ? 1.0 : 0.0, 1.0});
    return {unit.to_vector(), rotational ? motion.theta : motion.d};
}

}

matrix3 skew_symmetric(const Eigen::Vector3d &v)
{
    return cartan::hat(v);
}

Eigen::Vector3d from_skew_symmetric(const matrix3 &m)
{
    return cartan::vee(m);
}

expected<adjoint, refusal> adjoint_matrix_from_rotation_position(const rotation &r, const Eigen::Vector3d &p)
{
    const auto rot = lie_rotation::from_matrix(r);
    if(!rot.has_value())
        return unexpected(refusal::degenerate);

    return adjoint(lie_transform(rot.value(), p).adjoint());
}

expected<adjoint, refusal> adjoint_matrix_from_transform(const transform &tf)
{
    const auto pose = lie_transform::from_matrix(tf);
    if(!pose.has_value())
        return unexpected(refusal::degenerate);

    return adjoint(pose.value().adjoint());
}

expected<twist, refusal> adjoint_map(const twist &t, const transform &tf)
{
    const auto pose = lie_transform::from_matrix(tf);
    if(!pose.has_value())
        return unexpected(refusal::degenerate);

    return twist(pose.value().adjoint() * t);
}

twist twist_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    return lie_twist{w, v}.to_vector();
}

expected<twist, refusal> twist_from_screw(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h, double angular_velocity)
{
    const expected<screw_axis, refusal> axis = screw_axis_from_point_direction_pitch(q, s, h);
    if(!axis)
        return unexpected(axis.error());

    return twist(angular_velocity * *axis);
}

matrix4 twist_matrix_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    return cartan::hat(lie_twist{w, v}.to_vector());
}

matrix4 twist_matrix_from_twist(const twist &t)
{
    return cartan::hat(t);
}

screw_axis screw_axis_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    return unit_screw(lie_twist{w, v}).first;
}

expected<screw_axis, refusal> screw_axis_from_point_direction_pitch(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{
    if(s.isZero())
        return unexpected(refusal::degenerate);

    const cartan::screw_params<double> axis{q, s.normalized(), h};

    return screw_axis(cartan::from_screw_motion(lie_screw{axis, 1.0, 1.0}).to_vector());
}

rotation matrix_exponential_so3(const Eigen::Vector3d &w, double theta_radians)
{
    return lie_rotation::exp(theta_radians * w).matrix();
}

transform matrix_exponential_se3(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta_radians)
{
    return matrix_exponential_screw(lie_twist{w, v}.to_vector(), theta_radians);
}

transform matrix_exponential_screw(const screw_axis &s, double theta_radians)
{
    return lie_transform::exp(theta_radians * s).matrix();
}

expected<std::pair<Eigen::Vector3d, double>, refusal> matrix_logarithm_so3(const rotation &r)
{
    const auto rot = lie_rotation::from_matrix(r);
    if(!rot.has_value())
        return unexpected(refusal::degenerate);

    const cartan::axis_angle<double> aa = cartan::to_axis_angle(rot.value());

    return std::pair<Eigen::Vector3d, double>{aa.axis, aa.angle};
}

expected<std::pair<screw_axis, double>, refusal> matrix_logarithm_se3_rp(const rotation &r, const Eigen::Vector3d &p)
{
    const auto rot = lie_rotation::from_matrix(r);
    if(!rot.has_value())
        return unexpected(refusal::degenerate);

    return unit_screw(lie_twist::from_vector(lie_transform(rot.value(), p).log()));
}

expected<std::pair<screw_axis, double>, refusal> matrix_logarithm_se3(const transform &tf)
{
    const auto pose = lie_transform::from_matrix(tf);
    if(!pose.has_value())
        return unexpected(refusal::degenerate);

    return unit_screw(lie_twist::from_vector(pose.value().log()));
}

}
