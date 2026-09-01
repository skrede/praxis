#include "evaluation_tables.h"

#include "praxis/evaluation/comparators.h"

#include <utility>

namespace praxis::rigid_motion {

namespace {

using rotation_logarithm = std::pair<Eigen::Vector3d, double>;
using pose_logarithm     = std::pair<screw_axis, double>;

const screw_ops &screw_of(const void *value)
{
    return *static_cast<const screw_ops *>(value);
}

// A logarithm answers an axis and the amount turned about it, which names an element without being
// one, so the pair is unpacked and the elements it names are compared.
evaluation::residual rotation_logarithm_residual(const rotation_logarithm &held, const rotation_logarithm &against)
{
    return evaluation::log_up_to_branch_rotation_residual(held.first, held.second, against.first, against.second);
}

evaluation::residual pose_logarithm_residual(const pose_logarithm &held, const pose_logarithm &against)
{
    return evaluation::log_up_to_branch_pose_residual(held.first, held.second, against.first, against.second);
}

}

evaluation::case_result compare_matrix_exponential_so3(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const Eigen::Vector3d axis = drawn.unit_direction();
    const double radians       = drawn.angle_radians();
    const rotation held        = screw_of(first).matrix_exponential_so3(axis, radians);
    const rotation against     = screw_of(second).matrix_exponential_so3(axis, radians);

    return judged(evaluation::geodesic_residual(held, against), allowed);
}

evaluation::case_result compare_matrix_exponential_se3(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const Eigen::Vector3d angular = drawn.angular_part();
    const Eigen::Vector3d linear  = drawn.linear_part();
    const double radians          = drawn.angle_radians();
    const transform held          = screw_of(first).matrix_exponential_se3(angular, linear, radians);
    const transform against       = screw_of(second).matrix_exponential_se3(angular, linear, radians);

    return judged(evaluation::pose_residual(held, against), allowed);
}

evaluation::case_result compare_matrix_exponential_screw(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const screw_axis axis   = drawn.unit_twist();
    const double radians    = drawn.angle_radians();
    const transform held    = screw_of(first).matrix_exponential_screw(axis, radians);
    const transform against = screw_of(second).matrix_exponential_screw(axis, radians);

    return judged(evaluation::pose_residual(held, against), allowed);
}

evaluation::case_result compare_matrix_logarithm_so3(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const rotation turned                               = drawn.rotation_member();
    const expected<rotation_logarithm, refusal> held    = screw_of(first).matrix_logarithm_so3(turned);
    const expected<rotation_logarithm, refusal> against = screw_of(second).matrix_logarithm_so3(turned);

    return evaluation::agreed_or_refused(held, against, rotation_logarithm_residual, allowed);
}

evaluation::case_result compare_matrix_logarithm_se3_rp(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const rotation turned                           = drawn.rotation_member();
    const Eigen::Vector3d origin_metres             = drawn.position_metres();
    const expected<pose_logarithm, refusal> held    = screw_of(first).matrix_logarithm_se3_rp(turned, origin_metres);
    const expected<pose_logarithm, refusal> against = screw_of(second).matrix_logarithm_se3_rp(turned, origin_metres);

    return evaluation::agreed_or_refused(held, against, pose_logarithm_residual, allowed);
}

evaluation::case_result compare_matrix_logarithm_se3(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const transform pose                            = drawn.transform_member();
    const expected<pose_logarithm, refusal> held    = screw_of(first).matrix_logarithm_se3(pose);
    const expected<pose_logarithm, refusal> against = screw_of(second).matrix_logarithm_se3(pose);

    return evaluation::agreed_or_refused(held, against, pose_logarithm_residual, allowed);
}

}
