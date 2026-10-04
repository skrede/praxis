#include "group_membership.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include <cmath>

namespace praxis::rigid_motion {

namespace {

// Lynch & Park, Modern Robotics, Def. 3.20.
adjoint adjoint_representation(const rotation &r, const Eigen::Vector3d &p)
{
    adjoint ad           = adjoint::Zero();
    ad.block<3, 3>(0, 0) = r;
    ad.block<3, 3>(3, 0) = skew_symmetric(p) * r;
    ad.block<3, 3>(3, 3) = r;

    return ad;
}

}

// Lynch & Park, Modern Robotics, Def. 3.7, eq. (3.30).
matrix3 skew_symmetric(const Eigen::Vector3d &v)
{
    matrix3 m;
    m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;

    return m;
}

// Lynch & Park, Modern Robotics, Def. 3.7, eq. (3.30), read from the entries below the diagonal
// and the one above it at (0, 2).
Eigen::Vector3d from_skew_symmetric(const matrix3 &m)
{
    return {m(2, 1), m(0, 2), m(1, 0)};
}

expected<adjoint, refusal> adjoint_matrix_from_rotation_position(const rotation &r, const Eigen::Vector3d &p)
{
    if(!is_a_rotation(r))
        return unexpected(refusal::degenerate);

    return adjoint_representation(r, p);
}

expected<adjoint, refusal> adjoint_matrix_from_transform(const transform &tf)
{
    if(!is_a_rigid_motion(tf))
        return unexpected(refusal::degenerate);

    return adjoint_representation(tf.block<3, 3>(0, 0), tf.block<3, 1>(0, 3));
}

// Lynch & Park, Modern Robotics, Def. 3.20: V' = [Ad_T] V.
expected<twist, refusal> adjoint_map(const twist &t, const transform &tf)
{
    const expected<adjoint, refusal> ad = adjoint_matrix_from_transform(tf);
    if(!ad)
        return unexpected(ad.error());

    return twist(*ad * t);
}

// Lynch & Park, Modern Robotics, eq. (3.70): the angular half above the linear one.
twist twist_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    twist t;
    t << w, v;

    return t;
}

// Lynch & Park, Modern Robotics, sec. 3.3.2.2: V = S theta-dot.
expected<twist, refusal> twist_from_screw(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h, double angular_velocity)
{
    const expected<screw_axis, refusal> axis = screw_axis_from_point_direction_pitch(q, s, h);
    if(!axis)
        return unexpected(axis.error());

    return twist(angular_velocity * *axis);
}

// Lynch & Park, Modern Robotics, eq. (3.85).
matrix4 twist_matrix_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    matrix4 m           = matrix4::Zero();
    m.block<3, 3>(0, 0) = skew_symmetric(w);
    m.block<3, 1>(0, 3) = v;

    return m;
}

matrix4 twist_matrix_from_twist(const twist &t)
{
    return twist_matrix_from_angular_linear(t.head<3>(), t.tail<3>());
}

// Lynch & Park, Modern Robotics, sec. 3.3.2.2, cases (a) and (b); the zero twist names no axis and
// is answered as a translation along x.
screw_axis screw_axis_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    const twist t = twist_from_angular_linear(w, v);
    if(const double angular = w.norm(); angular != 0.0)
        return t / angular;
    if(const double linear = v.norm(); linear != 0.0)
        return t / linear;

    return twist_from_angular_linear(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
}

// Lynch & Park, Modern Robotics, Def. 3.24: v = -w x q + h w for the unit direction w.
expected<screw_axis, refusal> screw_axis_from_point_direction_pitch(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{
    if(s.isZero())
        return unexpected(refusal::degenerate);

    const Eigen::Vector3d w = s.normalized();

    return twist_from_angular_linear(w, -w.cross(q) + h * w);
}

// Lynch & Park, Modern Robotics, Prop. 3.11, eq. (3.51), over the exponential coordinates w theta:
// their norm is the angle and their direction the unit axis.
rotation matrix_exponential_so3(const Eigen::Vector3d &w, double theta_radians)
{
    const Eigen::Vector3d coordinates = w * theta_radians;
    const double angle                = coordinates.norm();
    if(angle == 0.0)
        return rotation::Identity();

    const matrix3 axis = skew_symmetric(coordinates / angle);

    return rotation::Identity() + std::sin(angle) * axis + (1.0 - std::cos(angle)) * axis * axis;
}

transform matrix_exponential_se3(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta_radians)
{
    return matrix_exponential_screw(twist_from_angular_linear(w, v), theta_radians);
}

// Lynch & Park, Modern Robotics, Prop. 3.25, eqs. (3.87)-(3.89), over S normalized by |w| as in
// sec. 3.3.2.2, case (a).
transform matrix_exponential_screw(const screw_axis &s, double theta_radians)
{
    transform tf        = transform::Identity();
    const double length = s.head<3>().norm();
    if(length == 0.0)
    {
        tf.block<3, 1>(0, 3) = s.tail<3>() * theta_radians;
        return tf;
    }

    const double angle      = theta_radians * length;
    const Eigen::Vector3d w = s.head<3>() / length;
    const matrix3 m         = skew_symmetric(w);
    const matrix3 g         = angle * matrix3::Identity() + (1.0 - std::cos(angle)) * m + (angle - std::sin(angle)) * m * m;

    tf.block<3, 3>(0, 0) = matrix_exponential_so3(w, angle);
    tf.block<3, 1>(0, 3) = g * s.tail<3>() / length;

    return tf;
}

}
