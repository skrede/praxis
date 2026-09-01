#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_BASELINE_FRAME_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_BASELINE_FRAME_H

#include "praxis/rigid_motion/frame.h"

// Every declaration below matches a slot of frame_ops by name and by signature, so composing the
// aggregate is a plain address-of.
namespace praxis::rigid_motion {

Eigen::Vector3d euler_from_rotation_matrix(const rotation &r, axis_order order);

rotation rotate_x(double radians);
rotation rotate_y(double radians);
rotation rotate_z(double radians);
rotation rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z);
rotation rotation_matrix_from_euler(const Eigen::Vector3d &e, axis_order order);
rotation rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double radians);
rotation rotation_matrix_from_transform(const transform &tf);

transform transformation_matrix_from_position(const Eigen::Vector3d &p);
transform transformation_matrix_from_rotation(const rotation &r);
transform transformation_matrix_from_rotation_position(const rotation &r, const Eigen::Vector3d &p);
transform inverse(const transform &tf);

}

#endif
