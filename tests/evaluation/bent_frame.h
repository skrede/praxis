#ifndef HPP_GUARD_PRAXIS_TESTS_EVALUATION_BENT_FRAME_H
#define HPP_GUARD_PRAXIS_TESTS_EVALUATION_BENT_FRAME_H

#include "bent_answer.h"

#include "praxis/rigid_motion/baseline/frame.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/capabilities.h"

#include <Eigen/Core>

#include <array>
#include <cstddef>

namespace praxis::fixture {

inline Eigen::Vector3d turned_euler_readout(const rotation &r, axis_order order)
{
    return Eigen::Vector3d(rigid_motion::euler_from_rotation_matrix(r, order) + bend_radians * Eigen::Vector3d::UnitX());
}

inline rotation turned_further_about_x(double radians)
{
    return rigid_motion::rotate_x(radians + bend_radians);
}

inline rotation turned_further_about_y(double radians)
{
    return rigid_motion::rotate_y(radians + bend_radians);
}

inline rotation turned_further_about_z(double radians)
{
    return rigid_motion::rotate_z(radians + bend_radians);
}

inline rotation turned_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z)
{
    return turned(rigid_motion::rotation_matrix_from_frame_axes(x, y, z));
}

inline rotation turned_from_euler(const Eigen::Vector3d &e, axis_order order)
{
    return turned(rigid_motion::rotation_matrix_from_euler(e, order));
}

inline rotation turned_from_axis_angle(const Eigen::Vector3d &axis, double radians)
{
    return turned(rigid_motion::rotation_matrix_from_axis_angle(axis, radians));
}

inline rotation turned_from_transform(const transform &tf)
{
    return turned(rigid_motion::rotation_matrix_from_transform(tf));
}

inline transform displaced_from_position(const Eigen::Vector3d &p)
{
    return displaced(rigid_motion::transformation_matrix_from_position(p));
}

inline transform turned_from_rotation(const rotation &r)
{
    return turned(rigid_motion::transformation_matrix_from_rotation(r));
}

inline transform displaced_from_rotation_position(const rotation &r, const Eigen::Vector3d &p)
{
    return displaced(rigid_motion::transformation_matrix_from_rotation_position(r, p));
}

inline transform displaced_inverse(const transform &tf)
{
    return displaced(rigid_motion::inverse(tf));
}

// The rows are in the enumerator order of frame_slot, and each one binds the slot its position
// names and nothing else.
inline constexpr std::array frame_bends{
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.frame.euler_from_rotation_matrix = &turned_euler_readout; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.frame.rotate_x = &turned_further_about_x; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.frame.rotate_y = &turned_further_about_y; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.frame.rotate_z = &turned_further_about_z; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.frame.rotation_matrix_from_frame_axes = &turned_from_frame_axes; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.frame.rotation_matrix_from_euler = &turned_from_euler; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.frame.rotation_matrix_from_axis_angle = &turned_from_axis_angle; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.frame.rotation_matrix_from_transform = &turned_from_transform; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.frame.transformation_matrix_from_position = &displaced_from_position; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.frame.transformation_matrix_from_rotation = &turned_from_rotation; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.frame.transformation_matrix_from_rotation_position = &displaced_from_rotation_position; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.frame.inverse = &displaced_inverse; }},
};

static_assert(frame_bends.size() == static_cast<std::size_t>(rigid_motion::frame_slot::count));

}

#endif
