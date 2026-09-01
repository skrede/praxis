#include "evaluation_tables.h"

#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/axis_order.h"

#include "praxis/evaluation/comparators.h"

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <cstdint>

namespace praxis::rigid_motion {

namespace {

const frame_ops &frame_of(const void *value)
{
    return *static_cast<const frame_ops *>(value);
}

rotation turn_about(std::uint8_t axis, double radians)
{
    return Eigen::AngleAxisd(radians, Eigen::Vector3d::Unit(axis)).toRotationMatrix();
}

// A rotation triple names an element rather than being one, and away from the singular
// configurations two triples name the same element, so the triples are composed and the elements
// compared. Each angle turns about the axis its position in the order names, outermost first, which
// is the convention the extraction is written against.
rotation composed_from_euler_radians(const Eigen::Vector3d &radians, axis_order order)
{
    const std::array<std::uint8_t, 3> axes = axis_indices(order);

    return turn_about(axes[0], radians[0]) * turn_about(axes[1], radians[1]) * turn_about(axes[2], radians[2]);
}

// The rows are in the enumerator order of frame_slot, and each name is spelled exactly as the
// descriptor table spells it.
constexpr std::array frame_table{
        evaluation::slot_evaluation{"frame.euler_from_rotation_matrix", evaluation::residual_kind::log_up_to_branch,
                                    evaluation::tolerance_of(evaluation::residual_kind::log_up_to_branch),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const rotation turned              = drawn.rotation_member();
                                        const axis_order order             = static_cast<axis_order>(drawn.axis_order_index());
                                        const rotation held                = composed_from_euler_radians(frame_of(first).euler_from_rotation_matrix(turned, order), order);
                                        const rotation against             = composed_from_euler_radians(frame_of(second).euler_from_rotation_matrix(turned, order), order);
                                        const evaluation::residual between = evaluation::geodesic_residual(held, against);

                                        return judged(evaluation::residual{evaluation::residual_kind::log_up_to_branch, between.magnitude, 0.0}, allowed);
                                    }},
        evaluation::slot_evaluation{"frame.rotate_x", evaluation::residual_kind::geodesic, evaluation::tolerance_of(evaluation::residual_kind::geodesic),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const double radians = drawn.angle_radians();

                                        return judged(evaluation::geodesic_residual(frame_of(first).rotate_x(radians), frame_of(second).rotate_x(radians)), allowed);
                                    }},
        evaluation::slot_evaluation{"frame.rotate_y", evaluation::residual_kind::geodesic, evaluation::tolerance_of(evaluation::residual_kind::geodesic),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const double radians = drawn.angle_radians();

                                        return judged(evaluation::geodesic_residual(frame_of(first).rotate_y(radians), frame_of(second).rotate_y(radians)), allowed);
                                    }},
        evaluation::slot_evaluation{"frame.rotate_z", evaluation::residual_kind::geodesic, evaluation::tolerance_of(evaluation::residual_kind::geodesic),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const double radians = drawn.angle_radians();

                                        return judged(evaluation::geodesic_residual(frame_of(first).rotate_z(radians), frame_of(second).rotate_z(radians)), allowed);
                                    }},
        evaluation::slot_evaluation{"frame.rotation_matrix_from_frame_axes", evaluation::residual_kind::geodesic, evaluation::tolerance_of(evaluation::residual_kind::geodesic),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const rotation triple  = drawn.orthonormal_triple();
                                        const rotation held    = frame_of(first).rotation_matrix_from_frame_axes(triple.col(0), triple.col(1), triple.col(2));
                                        const rotation against = frame_of(second).rotation_matrix_from_frame_axes(triple.col(0), triple.col(1), triple.col(2));

                                        return judged(evaluation::geodesic_residual(held, against), allowed);
                                    }},
        evaluation::slot_evaluation{"frame.rotation_matrix_from_euler", evaluation::residual_kind::geodesic, evaluation::tolerance_of(evaluation::residual_kind::geodesic),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const Eigen::Vector3d radians = drawn.euler_triple_radians();
                                        const axis_order order        = static_cast<axis_order>(drawn.axis_order_index());
                                        const rotation held           = frame_of(first).rotation_matrix_from_euler(radians, order);
                                        const rotation against        = frame_of(second).rotation_matrix_from_euler(radians, order);

                                        return judged(evaluation::geodesic_residual(held, against), allowed);
                                    }},
        evaluation::slot_evaluation{"frame.rotation_matrix_from_axis_angle", evaluation::residual_kind::geodesic, evaluation::tolerance_of(evaluation::residual_kind::geodesic),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const Eigen::Vector3d axis = drawn.unit_direction();
                                        const double radians       = drawn.angle_radians();
                                        const rotation held        = frame_of(first).rotation_matrix_from_axis_angle(axis, radians);
                                        const rotation against     = frame_of(second).rotation_matrix_from_axis_angle(axis, radians);

                                        return judged(evaluation::geodesic_residual(held, against), allowed);
                                    }},
        evaluation::slot_evaluation{"frame.rotation_matrix_from_transform", evaluation::residual_kind::geodesic, evaluation::tolerance_of(evaluation::residual_kind::geodesic),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const transform pose   = drawn.transform_member();
                                        const rotation held    = frame_of(first).rotation_matrix_from_transform(pose);
                                        const rotation against = frame_of(second).rotation_matrix_from_transform(pose);

                                        return judged(evaluation::geodesic_residual(held, against), allowed);
                                    }},
        evaluation::slot_evaluation{"frame.transformation_matrix_from_position", evaluation::residual_kind::pose, evaluation::tolerance_of(evaluation::residual_kind::pose),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const Eigen::Vector3d origin_metres = drawn.position_metres();
                                        const transform held                = frame_of(first).transformation_matrix_from_position(origin_metres);
                                        const transform against             = frame_of(second).transformation_matrix_from_position(origin_metres);

                                        return judged(evaluation::pose_residual(held, against), allowed);
                                    }},
        evaluation::slot_evaluation{"frame.transformation_matrix_from_rotation", evaluation::residual_kind::pose, evaluation::tolerance_of(evaluation::residual_kind::pose),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const rotation turned   = drawn.rotation_member();
                                        const transform held    = frame_of(first).transformation_matrix_from_rotation(turned);
                                        const transform against = frame_of(second).transformation_matrix_from_rotation(turned);

                                        return judged(evaluation::pose_residual(held, against), allowed);
                                    }},
        evaluation::slot_evaluation{"frame.transformation_matrix_from_rotation_position", evaluation::residual_kind::pose, evaluation::tolerance_of(evaluation::residual_kind::pose),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const rotation turned               = drawn.rotation_member();
                                        const Eigen::Vector3d origin_metres = drawn.position_metres();
                                        const transform held                = frame_of(first).transformation_matrix_from_rotation_position(turned, origin_metres);
                                        const transform against             = frame_of(second).transformation_matrix_from_rotation_position(turned, origin_metres);

                                        return judged(evaluation::pose_residual(held, against), allowed);
                                    }},
        evaluation::slot_evaluation{"frame.inverse", evaluation::residual_kind::pose, evaluation::tolerance_of(evaluation::residual_kind::pose),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const transform pose    = drawn.transform_member();
                                        const transform held    = frame_of(first).inverse(pose);
                                        const transform against = frame_of(second).inverse(pose);

                                        return judged(evaluation::pose_residual(held, against), allowed);
                                    }},
};

static_assert(frame_table.size() == static_cast<std::size_t>(frame_slot::count));

constexpr evaluation::capability_evaluations<frame_ops> evaluated_frames{"rigid_motion", frame_table};

}

const evaluation::capability_evaluations<frame_ops> &frame_evaluations()
{
    return evaluated_frames;
}

}
