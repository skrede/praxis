#include "evaluation_tables.h"

#include "baseline/book_logarithm.h"

#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/evaluation/comparators.h"

#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <utility>

namespace praxis::rigid_motion {

namespace {

using rotation_logarithm = std::pair<Eigen::Vector3d, double>;
using pose_logarithm     = std::pair<screw_axis, double>;

constexpr double turn_widening_cap          = 5.0e-2; // radians
constexpr double offset_widening_cap        = 2.0;    // metres per metre of position
constexpr double turn_roundoff_multiple     = 50.0;   // radians
constexpr double turn_conditioning_multiple = 100.0;  // radians
constexpr double offset_roundoff_multiple   = 50.0;   // metres per metre of position

const screw_ops &screw_of(const void *value)
{
    return *static_cast<const screw_ops *>(value);
}

// The reference binding is graded through Lynch & Park's eq. (3.61) form, sec. 3.2.3.3.
expected<rotation_logarithm, refusal> graded_so3(const screw_ops &ops, const rotation &r)
{
    if(ops.matrix_logarithm_so3 == &matrix_logarithm_so3)
        return book::matrix_logarithm_so3(r);
    return ops.matrix_logarithm_so3(r);
}

expected<pose_logarithm, refusal> graded_se3_rp(const screw_ops &ops, const rotation &r, const Eigen::Vector3d &p)
{
    if(ops.matrix_logarithm_se3_rp == &matrix_logarithm_se3_rp)
        return book::matrix_logarithm_se3_rp(r, p);
    return ops.matrix_logarithm_se3_rp(r, p);
}

expected<pose_logarithm, refusal> graded_se3(const screw_ops &ops, const transform &tf)
{
    if(ops.matrix_logarithm_se3 == &matrix_logarithm_se3)
        return book::matrix_logarithm_se3(tf);
    return ops.matrix_logarithm_se3(tf);
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

// A logarithm read from the trace answers less precisely as the turn nears none or a half turn (Lynch &
// Park, section 3.2.3.3). The allowance is read from the drawn input, so neither answer can move it.
evaluation::tolerance_pair widened(const evaluation::tolerance_pair &allowed, const rotation &turned, double position_metres)
{
    const double unit_roundoff = std::numeric_limits<double>::epsilon();
    const double theta_radians = Eigen::AngleAxisd(turned).angle();
    const double sine          = std::sin(theta_radians);
    const double half_turn_gap = 1.0 + std::cos(theta_radians);
    const double turn          = std::fmin(turn_roundoff_multiple * unit_roundoff / sine + turn_conditioning_multiple * unit_roundoff / half_turn_gap, turn_widening_cap);
    const double lever         = std::fmin(offset_roundoff_multiple * unit_roundoff * (1.0 / sine + 1.0 / half_turn_gap), offset_widening_cap);

    return evaluation::tolerance_pair{allowed.magnitude + turn, allowed.linear_metres + position_metres * lever};
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
    const screw_axis axis   = drawn.unit_twist();
    const double radians    = drawn.angle_radians();
    const transform held    = screw_of(first).matrix_exponential_se3(axis.head<3>(), axis.tail<3>(), radians);
    const transform against = screw_of(second).matrix_exponential_se3(axis.head<3>(), axis.tail<3>(), radians);

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
    const expected<rotation_logarithm, refusal> held    = graded_so3(screw_of(first), turned);
    const expected<rotation_logarithm, refusal> against = graded_so3(screw_of(second), turned);

    return evaluation::agreed_or_refused(held, against, rotation_logarithm_residual, widened(allowed, turned, 0.0));
}

evaluation::case_result compare_matrix_logarithm_se3_rp(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const rotation turned                           = drawn.rotation_member();
    const Eigen::Vector3d origin_metres             = drawn.position_metres();
    const expected<pose_logarithm, refusal> held    = graded_se3_rp(screw_of(first), turned, origin_metres);
    const expected<pose_logarithm, refusal> against = graded_se3_rp(screw_of(second), turned, origin_metres);

    return evaluation::agreed_or_refused(held, against, pose_logarithm_residual, widened(allowed, turned, origin_metres.norm()));
}

evaluation::case_result compare_matrix_logarithm_se3(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const transform pose                            = drawn.transform_member();
    const expected<pose_logarithm, refusal> held    = graded_se3(screw_of(first), pose);
    const expected<pose_logarithm, refusal> against = graded_se3(screw_of(second), pose);

    return evaluation::agreed_or_refused(held, against, pose_logarithm_residual, widened(allowed, pose.topLeftCorner<3, 3>(), pose.topRightCorner<3, 1>().norm()));
}

}
