#ifndef HPP_GUARD_PRAXIS_TESTS_EVALUATION_BEND_VISIBILITY_H
#define HPP_GUARD_PRAXIS_TESTS_EVALUATION_BEND_VISIBILITY_H

#include "bent_answer.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/evaluation/generation.h"

#include <Eigen/Core>

#include <array>
#include <cstddef>

// Each row calls one slot on both aggregates over one drawn input and answers whether the two came
// back with different answers. The roles drawn are the roles that slot's comparison draws, so a
// bend the facility cannot see is reported here rather than counted as a slot that agreed.
namespace praxis::fixture {

inline constexpr std::array frame_bend_probes{
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   {
                       return answers_differ(held.frame.euler_from_rotation_matrix, bent.frame.euler_from_rotation_matrix, drawn.rotation_member(),
                                             static_cast<axis_order>(drawn.axis_order_index()));
                   }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.frame.rotate_x, bent.frame.rotate_x, drawn.angle_radians()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.frame.rotate_y, bent.frame.rotate_y, drawn.angle_radians()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.frame.rotate_z, bent.frame.rotate_z, drawn.angle_radians()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   {
                       const rotation triple = drawn.orthonormal_triple();

                       return answers_differ(held.frame.rotation_matrix_from_frame_axes, bent.frame.rotation_matrix_from_frame_axes, triple.col(0), triple.col(1), triple.col(2));
                   }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   {
                       return answers_differ(held.frame.rotation_matrix_from_euler, bent.frame.rotation_matrix_from_euler, drawn.euler_triple_radians(),
                                             static_cast<axis_order>(drawn.axis_order_index()));
                   }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.frame.rotation_matrix_from_axis_angle, bent.frame.rotation_matrix_from_axis_angle, drawn.unit_direction(), drawn.angle_radians()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.frame.rotation_matrix_from_transform, bent.frame.rotation_matrix_from_transform, drawn.transform_member()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.frame.transformation_matrix_from_position, bent.frame.transformation_matrix_from_position, drawn.position_metres()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.frame.transformation_matrix_from_rotation, bent.frame.transformation_matrix_from_rotation, drawn.rotation_member()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   {
                       return answers_differ(held.frame.transformation_matrix_from_rotation_position, bent.frame.transformation_matrix_from_rotation_position, drawn.rotation_member(),
                                             drawn.position_metres());
                   }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.frame.inverse, bent.frame.inverse, drawn.transform_member()); }},
};

inline constexpr std::array screw_bend_probes{
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.screw.skew_symmetric, bent.screw.skew_symmetric, drawn.angular_part()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.screw.from_skew_symmetric, bent.screw.from_skew_symmetric, drawn.skew_symmetric_member()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   {
                       return answers_differ(held.screw.adjoint_matrix_from_rotation_position, bent.screw.adjoint_matrix_from_rotation_position, drawn.rotation_member(),
                                             drawn.position_metres());
                   }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.screw.adjoint_matrix_from_transform, bent.screw.adjoint_matrix_from_transform, drawn.transform_member()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.screw.adjoint_map, bent.screw.adjoint_map, drawn.twist_member(), drawn.transform_member()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.screw.twist_from_angular_linear, bent.screw.twist_from_angular_linear, drawn.angular_part(), drawn.linear_part()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   {
                       return answers_differ(held.screw.twist_from_screw, bent.screw.twist_from_screw, drawn.position_metres(), drawn.unit_direction(), drawn.pitch(),
                                             drawn.angle_radians());
                   }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.screw.twist_matrix_from_angular_linear, bent.screw.twist_matrix_from_angular_linear, drawn.angular_part(), drawn.linear_part()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.screw.twist_matrix_from_twist, bent.screw.twist_matrix_from_twist, drawn.twist_member()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.screw.screw_axis_from_angular_linear, bent.screw.screw_axis_from_angular_linear, drawn.angular_part(), drawn.linear_part()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   {
                       return answers_differ(held.screw.screw_axis_from_point_direction_pitch, bent.screw.screw_axis_from_point_direction_pitch, drawn.position_metres(),
                                             drawn.unit_direction(), drawn.pitch());
                   }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.screw.matrix_exponential_so3, bent.screw.matrix_exponential_so3, drawn.unit_direction(), drawn.angle_radians()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.screw.matrix_exponential_se3, bent.screw.matrix_exponential_se3, drawn.angular_part(), drawn.linear_part(), drawn.angle_radians()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.screw.matrix_exponential_screw, bent.screw.matrix_exponential_screw, drawn.unit_twist(), drawn.angle_radians()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.screw.matrix_logarithm_so3, bent.screw.matrix_logarithm_so3, drawn.rotation_member()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.screw.matrix_logarithm_se3_rp, bent.screw.matrix_logarithm_se3_rp, drawn.rotation_member(), drawn.position_metres()); }},
        bend_probe{[](const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
                   { return answers_differ(held.screw.matrix_logarithm_se3, bent.screw.matrix_logarithm_se3, drawn.transform_member()); }},
};

static_assert(frame_bend_probes.size() == static_cast<std::size_t>(rigid_motion::frame_slot::count));
static_assert(screw_bend_probes.size() == static_cast<std::size_t>(rigid_motion::screw_slot::count));

}

#endif
