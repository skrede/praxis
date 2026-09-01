#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_FRAME_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_FRAME_H

#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/axis_order.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace praxis::rigid_motion::inert {

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

namespace praxis::rigid_motion {

// Declaration order is frozen: a designated initializer must name members in declaration order, so
// reordering a slot breaks every project that already composes this aggregate. Appending is safe.
struct frame_ops
{
    Eigen::Vector3d (*euler_from_rotation_matrix)(const rotation &r, axis_order order) = &inert::euler_from_rotation_matrix;

    rotation (*rotate_x)(double radians)                                                                                      = &inert::rotate_x;
    rotation (*rotate_y)(double radians)                                                                                      = &inert::rotate_y;
    rotation (*rotate_z)(double radians)                                                                                      = &inert::rotate_z;
    rotation (*rotation_matrix_from_frame_axes)(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z) = &inert::rotation_matrix_from_frame_axes;
    rotation (*rotation_matrix_from_euler)(const Eigen::Vector3d &e, axis_order order)                                        = &inert::rotation_matrix_from_euler;
    rotation (*rotation_matrix_from_axis_angle)(const Eigen::Vector3d &axis, double radians)                                  = &inert::rotation_matrix_from_axis_angle;
    rotation (*rotation_matrix_from_transform)(const transform &tf)                                                           = &inert::rotation_matrix_from_transform;

    transform (*transformation_matrix_from_position)(const Eigen::Vector3d &p)                             = &inert::transformation_matrix_from_position;
    transform (*transformation_matrix_from_rotation)(const rotation &r)                                    = &inert::transformation_matrix_from_rotation;
    transform (*transformation_matrix_from_rotation_position)(const rotation &r, const Eigen::Vector3d &p) = &inert::transformation_matrix_from_rotation_position;
    transform (*inverse)(const transform &tf)                                                              = &inert::inverse;
};

}

#endif
