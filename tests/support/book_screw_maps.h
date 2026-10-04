#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_SCREW_MAPS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_SCREW_MAPS_H

#include "praxis/rigid_motion/screw.h"

#include "praxis/evaluation/tolerance.h"

#include <cmath>
#include <numbers>
#include <utility>
#include <algorithm>

// Lynch & Park, Modern Robotics, chapter 3, transcribed from the text alone: formulas as printed,
// exact case tests, the arc cosine's argument clamped to its domain and nothing else, the eq. (3.61)
// axis left as computed. The refusal ahead of a formula is the slot's contract, not the text's.
namespace praxis::tests::book {

using rotation_reading = std::pair<Eigen::Vector3d, double>;
using pose_reading     = std::pair<screw_axis, double>;

inline bool is_a_rotation(const rotation &r)
{
    return r.allFinite() && is_approx_equal(rotation(r.transpose() * r), rotation::Identity()) && is_approx_equal(r.determinant(), 1.0);
}

inline bool is_rigid(const transform &tf)
{
    return tf.allFinite() && is_a_rotation(tf.block<3, 3>(0, 0)) && is_approx_equal((tf.row(3) - Eigen::RowVector4d::UnitW()).cwiseAbs().maxCoeff(), 0.0);
}

// eq. (3.30)
inline matrix3 skew_symmetric(const Eigen::Vector3d &w)
{
    matrix3 m;
    m << 0.0, -w(2), w(1), w(2), 0.0, -w(0), -w(1), w(0), 0.0;
    return m;
}

inline Eigen::Vector3d from_skew_symmetric(const matrix3 &m)
{
    return Eigen::Vector3d(m(2, 1), m(0, 2), m(1, 0));
}

// Def. 3.20: [Ad_T] = [R 0; [p]R R]
inline adjoint adjoint_of(const rotation &r, const Eigen::Vector3d &p)
{
    adjoint ad           = adjoint::Zero();
    ad.block<3, 3>(0, 0) = r;
    ad.block<3, 3>(3, 0) = skew_symmetric(p) * r;
    ad.block<3, 3>(3, 3) = r;
    return ad;
}

inline expected<adjoint, refusal> adjoint_matrix_from_rotation_position(const rotation &r, const Eigen::Vector3d &p)
{
    if(!p.allFinite() || !is_a_rotation(r))
        return unexpected(refusal::degenerate);
    return adjoint_of(r, p);
}

inline expected<adjoint, refusal> adjoint_matrix_from_transform(const transform &tf)
{
    if(!is_rigid(tf))
        return unexpected(refusal::degenerate);
    return adjoint_of(tf.block<3, 3>(0, 0), tf.block<3, 1>(0, 3));
}

// Def. 3.20: V' = [Ad_T] V
inline expected<twist, refusal> adjoint_map(const twist &t, const transform &tf)
{
    if(!is_rigid(tf))
        return unexpected(refusal::degenerate);
    return twist(adjoint_of(tf.block<3, 3>(0, 0), tf.block<3, 1>(0, 3)) * t);
}

// eq. (3.70)
inline twist twist_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    twist t;
    t << w, v;
    return t;
}

// sec. 3.3.2.2: V = (s theta-dot, -s theta-dot x q + h s theta-dot)
inline expected<twist, refusal> twist_from_screw(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h, double angular_velocity)
{
    if(s.isZero())
        return unexpected(refusal::degenerate);
    const Eigen::Vector3d w = s * angular_velocity;
    return twist_from_angular_linear(w, Eigen::Vector3d(-w.cross(q) + h * w));
}

// eq. (3.85)
inline matrix4 twist_matrix_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    matrix4 m           = matrix4::Zero();
    m.block<3, 3>(0, 0) = skew_symmetric(w);
    m.block<3, 1>(0, 3) = v;
    return m;
}

inline matrix4 twist_matrix_from_twist(const twist &t)
{
    return twist_matrix_from_angular_linear(t.head<3>(), t.tail<3>());
}

// sec. 3.3.2.2: S = V/||omega||, or S = V/||v|| where omega = 0
inline screw_axis screw_axis_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    const twist t = twist_from_angular_linear(w, v);
    return w.norm() != 0.0 ? screw_axis(t / w.norm()) : screw_axis(t / v.norm());
}

// sec. 3.3.2.2: S = (s, -s x q + h s)
inline expected<screw_axis, refusal> screw_axis_from_point_direction_pitch(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{
    if(s.isZero())
        return unexpected(refusal::degenerate);
    return twist_from_angular_linear(s, Eigen::Vector3d(-s.cross(q) + h * s));
}

// Prop. 3.11, eq. (3.51)
inline rotation matrix_exponential_so3(const Eigen::Vector3d &w, double theta)
{
    const matrix3 m = skew_symmetric(w);
    return rotation::Identity() + std::sin(theta) * m + (1.0 - std::cos(theta)) * m * m;
}

// Prop. 3.25, eqs. (3.88) and (3.89)
inline transform matrix_exponential_screw(const screw_axis &s, double theta)
{
    const Eigen::Vector3d w = s.head<3>();
    const matrix3 m         = skew_symmetric(w);
    transform tf            = transform::Identity();
    if(w.norm() == 0.0)
        tf.block<3, 1>(0, 3) = s.tail<3>() * theta;
    else
    {
        tf.block<3, 3>(0, 0) = matrix_exponential_so3(w, theta);
        tf.block<3, 1>(0, 3) = (matrix3::Identity() * theta + (1.0 - std::cos(theta)) * m + (theta - std::sin(theta)) * m * m) * s.tail<3>();
    }
    return tf;
}

inline transform matrix_exponential_se3(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta)
{
    return matrix_exponential_screw(twist_from_angular_linear(w, v), theta);
}

// sec. 3.2.3.3, eqs. (3.58)-(3.61)
inline rotation_reading rotation_logarithm(const rotation &r)
{
    const double pi = std::numbers::pi;
    if(r == rotation::Identity())
        return {Eigen::Vector3d::Zero(), 0.0};
    if(r.trace() == -1.0 && 1.0 + r(2, 2) != 0.0)
        return {Eigen::Vector3d(r(0, 2), r(1, 2), 1.0 + r(2, 2)) / std::sqrt(2.0 * (1.0 + r(2, 2))), pi};
    if(r.trace() == -1.0 && 1.0 + r(1, 1) != 0.0)
        return {Eigen::Vector3d(r(0, 1), 1.0 + r(1, 1), r(2, 1)) / std::sqrt(2.0 * (1.0 + r(1, 1))), pi};
    if(r.trace() == -1.0)
        return {Eigen::Vector3d(1.0 + r(0, 0), r(1, 0), r(2, 0)) / std::sqrt(2.0 * (1.0 + r(0, 0))), pi};
    const double theta = std::acos(std::clamp((r.trace() - 1.0) / 2.0, -1.0, 1.0));
    return {from_skew_symmetric((r - r.transpose()) / (2.0 * std::sin(theta))), theta};
}

// sec. 3.3.3.2, eqs. (3.91)-(3.92)
inline pose_reading pose_logarithm(const rotation &r, const Eigen::Vector3d &p)
{
    if(r == rotation::Identity())
        return {twist_from_angular_linear(Eigen::Vector3d::Zero(), p / p.norm()), p.norm()};
    const auto [w, theta]   = rotation_logarithm(r);
    const matrix3 m         = skew_symmetric(w);
    const matrix3 g_inverse = matrix3::Identity() / theta - 0.5 * m + (1.0 / theta - 0.5 / std::tan(theta / 2.0)) * m * m;
    return {twist_from_angular_linear(w, g_inverse * p), theta};
}

inline expected<rotation_reading, refusal> matrix_logarithm_so3(const rotation &r)
{
    if(!is_a_rotation(r))
        return unexpected(refusal::degenerate);
    return rotation_logarithm(r);
}

inline expected<pose_reading, refusal> matrix_logarithm_se3_rp(const rotation &r, const Eigen::Vector3d &p)
{
    if(!p.allFinite() || !is_a_rotation(r))
        return unexpected(refusal::degenerate);
    return pose_logarithm(r, p);
}

inline expected<pose_reading, refusal> matrix_logarithm_se3(const transform &tf)
{
    if(!is_rigid(tf))
        return unexpected(refusal::degenerate);
    return pose_logarithm(tf.block<3, 3>(0, 0), tf.block<3, 1>(0, 3));
}

}

#endif
