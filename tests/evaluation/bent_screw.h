#ifndef HPP_GUARD_PRAXIS_TESTS_EVALUATION_BENT_SCREW_H
#define HPP_GUARD_PRAXIS_TESTS_EVALUATION_BENT_SCREW_H

#include "bent_answer.h"

#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/capabilities.h"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <utility>

namespace praxis::fixture {

inline matrix3 moved_skew(const Eigen::Vector3d &v)
{
    return moved(rigid_motion::skew_symmetric(v));
}

inline Eigen::Vector3d moved_skew_readout(const matrix3 &m)
{
    return moved(rigid_motion::from_skew_symmetric(m));
}

inline expected<adjoint, refusal> moved_adjoint_from_rotation_position(const rotation &r, const Eigen::Vector3d &p)
{
    return moved_or_refused(rigid_motion::adjoint_matrix_from_rotation_position(r, p), [](const adjoint &a) { return moved(a); });
}

inline expected<adjoint, refusal> moved_adjoint_from_transform(const transform &tf)
{
    return moved_or_refused(rigid_motion::adjoint_matrix_from_transform(tf), [](const adjoint &a) { return moved(a); });
}

inline expected<twist, refusal> moved_adjoint_image(const twist &t, const transform &tf)
{
    return moved_or_refused(rigid_motion::adjoint_map(t, tf), [](const twist &moved_twist) { return moved(moved_twist); });
}

inline twist moved_twist_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    return moved(rigid_motion::twist_from_angular_linear(w, v));
}

inline expected<twist, refusal> moved_twist_from_screw(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h, double angular_velocity)
{
    return moved_or_refused(rigid_motion::twist_from_screw(q, s, h, angular_velocity), [](const twist &t) { return moved(t); });
}

inline matrix4 moved_twist_matrix_from_angular_linear(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    return moved(rigid_motion::twist_matrix_from_angular_linear(w, v));
}

inline matrix4 moved_twist_matrix(const twist &t)
{
    return moved(rigid_motion::twist_matrix_from_twist(t));
}

inline screw_axis moved_screw_axis(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    return moved_in_the_angular_half(rigid_motion::screw_axis_from_angular_linear(w, v));
}

inline expected<screw_axis, refusal> moved_axis_from_point(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{
    return moved_or_refused(rigid_motion::screw_axis_from_point_direction_pitch(q, s, h), [](const screw_axis &axis) { return moved_in_the_angular_half(axis); });
}

inline rotation turned_exponential_so3(const Eigen::Vector3d &w, double theta_radians)
{
    return turned(rigid_motion::matrix_exponential_so3(w, theta_radians));
}

inline transform displaced_exponential_se3(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta_radians)
{
    return displaced(rigid_motion::matrix_exponential_se3(w, v, theta_radians));
}

inline transform displaced_exponential_screw(const screw_axis &s, double theta_radians)
{
    return displaced(rigid_motion::matrix_exponential_screw(s, theta_radians));
}

inline expected<std::pair<Eigen::Vector3d, double>, refusal> turned_logarithm_so3(const rotation &r)
{
    return moved_or_refused(rigid_motion::matrix_logarithm_so3(r),
                            [](const std::pair<Eigen::Vector3d, double> &read) { return std::pair<Eigen::Vector3d, double>{read.first, read.second + bend_radians}; });
}

inline expected<std::pair<screw_axis, double>, refusal> displaced_logarithm_se3_rp(const rotation &r, const Eigen::Vector3d &p)
{
    return moved_or_refused(rigid_motion::matrix_logarithm_se3_rp(r, p),
                            [](const std::pair<screw_axis, double> &read) { return std::pair<screw_axis, double>{moved_in_the_linear_half(read.first), read.second}; });
}

inline expected<std::pair<screw_axis, double>, refusal> displaced_logarithm_se3(const transform &tf)
{
    return moved_or_refused(rigid_motion::matrix_logarithm_se3(tf),
                            [](const std::pair<screw_axis, double> &read) { return std::pair<screw_axis, double>{moved_in_the_linear_half(read.first), read.second}; });
}

// The rows are in the enumerator order of screw_slot, and each one binds the slot its position
// names and nothing else.
inline constexpr std::array screw_bends{
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.skew_symmetric = &moved_skew; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.from_skew_symmetric = &moved_skew_readout; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.adjoint_matrix_from_rotation_position = &moved_adjoint_from_rotation_position; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.adjoint_matrix_from_transform = &moved_adjoint_from_transform; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.adjoint_map = &moved_adjoint_image; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.twist_from_angular_linear = &moved_twist_from_angular_linear; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.twist_from_screw = &moved_twist_from_screw; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.twist_matrix_from_angular_linear = &moved_twist_matrix_from_angular_linear; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.twist_matrix_from_twist = &moved_twist_matrix; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.screw_axis_from_angular_linear = &moved_screw_axis; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.screw_axis_from_point_direction_pitch = &moved_axis_from_point; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.matrix_exponential_so3 = &turned_exponential_so3; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.matrix_exponential_se3 = &displaced_exponential_se3; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.matrix_exponential_screw = &displaced_exponential_screw; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.matrix_logarithm_so3 = &turned_logarithm_so3; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.matrix_logarithm_se3_rp = &displaced_logarithm_se3_rp; }},
        bend_applier{[](rigid_motion::capabilities &spatial) { spatial.screw.matrix_logarithm_se3 = &displaced_logarithm_se3; }},
};

static_assert(screw_bends.size() == static_cast<std::size_t>(rigid_motion::screw_slot::count));

}

#endif
