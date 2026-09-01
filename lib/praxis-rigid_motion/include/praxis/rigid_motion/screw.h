#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_SCREW_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_SCREW_H

#include "praxis/rigid_motion/types.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <utility>

namespace praxis::rigid_motion::inert {

matrix3 skew_symmetric(const Eigen::Vector3d &v);
Eigen::Vector3d from_skew_symmetric(const matrix3 &m);

expected<adjoint, refusal> adjoint_matrix_from_rotation_position(const rotation &r, const Eigen::Vector3d &p);
expected<adjoint, refusal> adjoint_matrix_from_transform(const transform &tf);
expected<twist, refusal> adjoint_map(const twist &t, const transform &tf);

twist twist_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v);
expected<twist, refusal> twist_from_screw(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h, double angular_velocity);
matrix4 twist_matrix_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v);
matrix4 twist_matrix_from_twist(const twist &t);

screw_axis screw_axis_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v);
expected<screw_axis, refusal> screw_axis_from_point_direction_pitch(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h);

rotation matrix_exponential_so3(const Eigen::Vector3d &w, double theta_radians);
transform matrix_exponential_se3(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta_radians);
transform matrix_exponential_screw(const screw_axis &s, double theta_radians);

expected<std::pair<Eigen::Vector3d, double>, refusal> matrix_logarithm_so3(const rotation &r);
expected<std::pair<screw_axis, double>, refusal> matrix_logarithm_se3_rp(const rotation &r, const Eigen::Vector3d &p);
expected<std::pair<screw_axis, double>, refusal> matrix_logarithm_se3(const transform &tf);

}

namespace praxis::rigid_motion {

// Declaration order is frozen: a designated initializer must name members in declaration order, so
// reordering a slot breaks every project that already composes this aggregate. Appending is safe.
struct screw_ops
{
    matrix3 (*skew_symmetric)(const Eigen::Vector3d &v)      = &inert::skew_symmetric;
    Eigen::Vector3d (*from_skew_symmetric)(const matrix3 &m) = &inert::from_skew_symmetric;

    expected<adjoint, refusal> (*adjoint_matrix_from_rotation_position)(const rotation &r, const Eigen::Vector3d &p) = &inert::adjoint_matrix_from_rotation_position;
    expected<adjoint, refusal> (*adjoint_matrix_from_transform)(const transform &tf)                                 = &inert::adjoint_matrix_from_transform;
    expected<twist, refusal> (*adjoint_map)(const twist &t, const transform &tf)                                     = &inert::adjoint_map;

    twist (*twist_from_angular_linear)(const Eigen::Vector3d &w, const Eigen::Vector3d &v)                                              = &inert::twist_from_angular_linear;
    expected<twist, refusal> (*twist_from_screw)(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h, double angular_velocity) = &inert::twist_from_screw;
    matrix4 (*twist_matrix_from_angular_linear)(const Eigen::Vector3d &w, const Eigen::Vector3d &v)                                     = &inert::twist_matrix_from_angular_linear;
    matrix4 (*twist_matrix_from_twist)(const twist &t)                                                                                  = &inert::twist_matrix_from_twist;

    screw_axis (*screw_axis_from_angular_linear)(const Eigen::Vector3d &w, const Eigen::Vector3d &v)                                     = &inert::screw_axis_from_angular_linear;
    expected<screw_axis, refusal> (*screw_axis_from_point_direction_pitch)(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h) = &inert::screw_axis_from_point_direction_pitch;

    rotation (*matrix_exponential_so3)(const Eigen::Vector3d &w, double theta_radians)                            = &inert::matrix_exponential_so3;
    transform (*matrix_exponential_se3)(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta_radians) = &inert::matrix_exponential_se3;
    transform (*matrix_exponential_screw)(const screw_axis &s, double theta_radians)                              = &inert::matrix_exponential_screw;

    expected<std::pair<Eigen::Vector3d, double>, refusal> (*matrix_logarithm_so3)(const rotation &r)                         = &inert::matrix_logarithm_so3;
    expected<std::pair<screw_axis, double>, refusal> (*matrix_logarithm_se3_rp)(const rotation &r, const Eigen::Vector3d &p) = &inert::matrix_logarithm_se3_rp;
    expected<std::pair<screw_axis, double>, refusal> (*matrix_logarithm_se3)(const transform &tf)                            = &inert::matrix_logarithm_se3;
};

}

#endif
