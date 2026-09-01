#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_BASELINE_SCREW_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_BASELINE_SCREW_H

#include "praxis/rigid_motion/screw.h"

#include <utility>

// Every declaration below matches a slot of screw_ops by name and by signature, so composing the
// aggregate is a plain address-of.
namespace praxis::rigid_motion {

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

#endif
