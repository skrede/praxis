#include "evaluation_tables.h"

#include "praxis/rigid_motion/slots.h"

#include "praxis/evaluation/comparators.h"

#include <array>
#include <cstddef>

namespace praxis::rigid_motion {

namespace {

const screw_ops &screw_of(const void *value)
{
    return *static_cast<const screw_ops *>(value);
}

// The rows are in the enumerator order of screw_slot, and each name is spelled exactly as the
// descriptor table spells it. A skew-symmetric matrix and a twist matrix are compared element-wise
// rather than as group elements: neither is in a group, so a geodesic angle or a pose discrepancy
// read over one is a number that means nothing and may still be small.
constexpr std::array screw_table{
        evaluation::slot_evaluation{"screw.skew_symmetric", evaluation::residual_kind::element_wise, evaluation::tolerance_of(evaluation::residual_kind::element_wise),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const Eigen::Vector3d angular = drawn.angular_part();
                                        const matrix3 held            = screw_of(first).skew_symmetric(angular);
                                        const matrix3 against         = screw_of(second).skew_symmetric(angular);

                                        return judged(evaluation::element_wise_residual(held, against), allowed);
                                    }},
        evaluation::slot_evaluation{"screw.from_skew_symmetric", evaluation::residual_kind::element_wise, evaluation::tolerance_of(evaluation::residual_kind::element_wise),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const matrix3 skew            = drawn.skew_symmetric_member();
                                        const Eigen::Vector3d held    = screw_of(first).from_skew_symmetric(skew);
                                        const Eigen::Vector3d against = screw_of(second).from_skew_symmetric(skew);

                                        return judged(evaluation::element_wise_residual(held, against), allowed);
                                    }},
        evaluation::slot_evaluation{"screw.adjoint_matrix_from_rotation_position", evaluation::residual_kind::element_wise,
                                    evaluation::tolerance_of(evaluation::residual_kind::element_wise),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const rotation turned                    = drawn.rotation_member();
                                        const Eigen::Vector3d origin_metres      = drawn.position_metres();
                                        const expected<adjoint, refusal> held    = screw_of(first).adjoint_matrix_from_rotation_position(turned, origin_metres);
                                        const expected<adjoint, refusal> against = screw_of(second).adjoint_matrix_from_rotation_position(turned, origin_metres);

                                        return evaluation::agreed_or_refused(held, against, evaluation::element_wise_residual, allowed);
                                    }},
        evaluation::slot_evaluation{"screw.adjoint_matrix_from_transform", evaluation::residual_kind::element_wise, evaluation::tolerance_of(evaluation::residual_kind::element_wise),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const transform pose                     = drawn.transform_member();
                                        const expected<adjoint, refusal> held    = screw_of(first).adjoint_matrix_from_transform(pose);
                                        const expected<adjoint, refusal> against = screw_of(second).adjoint_matrix_from_transform(pose);

                                        return evaluation::agreed_or_refused(held, against, evaluation::element_wise_residual, allowed);
                                    }},
        evaluation::slot_evaluation{"screw.adjoint_map", evaluation::residual_kind::element_wise, evaluation::tolerance_of(evaluation::residual_kind::element_wise),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const twist moved                      = drawn.twist_member();
                                        const transform pose                   = drawn.transform_member();
                                        const expected<twist, refusal> held    = screw_of(first).adjoint_map(moved, pose);
                                        const expected<twist, refusal> against = screw_of(second).adjoint_map(moved, pose);

                                        return evaluation::agreed_or_refused(held, against, evaluation::element_wise_residual, allowed);
                                    }},
        evaluation::slot_evaluation{"screw.twist_from_angular_linear", evaluation::residual_kind::element_wise, evaluation::tolerance_of(evaluation::residual_kind::element_wise),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const Eigen::Vector3d angular = drawn.angular_part();
                                        const Eigen::Vector3d linear  = drawn.linear_part();
                                        const twist held              = screw_of(first).twist_from_angular_linear(angular, linear);
                                        const twist against           = screw_of(second).twist_from_angular_linear(angular, linear);

                                        return judged(evaluation::element_wise_residual(held, against), allowed);
                                    }},
        evaluation::slot_evaluation{"screw.twist_from_screw", evaluation::residual_kind::element_wise, evaluation::tolerance_of(evaluation::residual_kind::element_wise),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const Eigen::Vector3d point_metres     = drawn.position_metres();
                                        const Eigen::Vector3d direction        = drawn.unit_direction();
                                        const double pitch                     = drawn.pitch();
                                        const double angular_velocity          = drawn.angle_radians();
                                        const expected<twist, refusal> held    = screw_of(first).twist_from_screw(point_metres, direction, pitch, angular_velocity);
                                        const expected<twist, refusal> against = screw_of(second).twist_from_screw(point_metres, direction, pitch, angular_velocity);

                                        return evaluation::agreed_or_refused(held, against, evaluation::element_wise_residual, allowed);
                                    }},
        evaluation::slot_evaluation{"screw.twist_matrix_from_angular_linear", evaluation::residual_kind::element_wise, evaluation::tolerance_of(evaluation::residual_kind::element_wise),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const Eigen::Vector3d angular = drawn.angular_part();
                                        const Eigen::Vector3d linear  = drawn.linear_part();
                                        const matrix4 held            = screw_of(first).twist_matrix_from_angular_linear(angular, linear);
                                        const matrix4 against         = screw_of(second).twist_matrix_from_angular_linear(angular, linear);

                                        return judged(evaluation::element_wise_residual(held, against), allowed);
                                    }},
        evaluation::slot_evaluation{"screw.twist_matrix_from_twist", evaluation::residual_kind::element_wise, evaluation::tolerance_of(evaluation::residual_kind::element_wise),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const twist moved     = drawn.twist_member();
                                        const matrix4 held    = screw_of(first).twist_matrix_from_twist(moved);
                                        const matrix4 against = screw_of(second).twist_matrix_from_twist(moved);

                                        return judged(evaluation::element_wise_residual(held, against), allowed);
                                    }},
        evaluation::slot_evaluation{"screw.screw_axis_from_angular_linear", evaluation::residual_kind::axis_up_to_sign,
                                    evaluation::tolerance_of(evaluation::residual_kind::axis_up_to_sign),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const Eigen::Vector3d angular = drawn.angular_part();
                                        const Eigen::Vector3d linear  = drawn.linear_part();
                                        const screw_axis held         = screw_of(first).screw_axis_from_angular_linear(angular, linear);
                                        const screw_axis against      = screw_of(second).screw_axis_from_angular_linear(angular, linear);

                                        return judged(evaluation::axis_up_to_sign_residual(held, against), allowed);
                                    }},
        evaluation::slot_evaluation{"screw.screw_axis_from_point_direction_pitch", evaluation::residual_kind::axis_up_to_sign,
                                    evaluation::tolerance_of(evaluation::residual_kind::axis_up_to_sign),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const Eigen::Vector3d point_metres          = drawn.position_metres();
                                        const Eigen::Vector3d direction             = drawn.unit_direction();
                                        const double pitch                          = drawn.pitch();
                                        const expected<screw_axis, refusal> held    = screw_of(first).screw_axis_from_point_direction_pitch(point_metres, direction, pitch);
                                        const expected<screw_axis, refusal> against = screw_of(second).screw_axis_from_point_direction_pitch(point_metres, direction, pitch);

                                        return evaluation::agreed_or_refused(held, against, evaluation::axis_up_to_sign_residual, allowed);
                                    }},
        evaluation::slot_evaluation{"screw.matrix_exponential_so3", evaluation::residual_kind::geodesic, evaluation::tolerance_of(evaluation::residual_kind::geodesic),
                                    &compare_matrix_exponential_so3},
        evaluation::slot_evaluation{"screw.matrix_exponential_se3", evaluation::residual_kind::pose, evaluation::tolerance_of(evaluation::residual_kind::pose),
                                    &compare_matrix_exponential_se3},
        evaluation::slot_evaluation{"screw.matrix_exponential_screw", evaluation::residual_kind::pose, evaluation::tolerance_of(evaluation::residual_kind::pose),
                                    &compare_matrix_exponential_screw},
        evaluation::slot_evaluation{"screw.matrix_logarithm_so3", evaluation::residual_kind::log_up_to_branch, evaluation::tolerance_of(evaluation::residual_kind::log_up_to_branch),
                                    &compare_matrix_logarithm_so3},
        evaluation::slot_evaluation{"screw.matrix_logarithm_se3_rp", evaluation::residual_kind::log_up_to_branch, evaluation::tolerance_of(evaluation::residual_kind::log_up_to_branch),
                                    &compare_matrix_logarithm_se3_rp},
        evaluation::slot_evaluation{"screw.matrix_logarithm_se3", evaluation::residual_kind::log_up_to_branch, evaluation::tolerance_of(evaluation::residual_kind::log_up_to_branch),
                                    &compare_matrix_logarithm_se3},
};

static_assert(screw_table.size() == static_cast<std::size_t>(screw_slot::count));

constexpr evaluation::capability_evaluations<screw_ops> evaluated_screws{"rigid_motion", screw_table};

}

const evaluation::capability_evaluations<screw_ops> &screw_evaluations()
{
    return evaluated_screws;
}

}
