#include "praxis/trajectory/baseline/path.h"

#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/evaluation/tolerance.h"

namespace praxis::trajectory {

namespace {

// Membership of SE(3): an orthonormal rotation of unit determinant over an affine bottom row. Both
// task-space paths premultiply by the start frame, so an endpoint outside the set produces an
// interpolated pose outside it even where the relative motion between the two endpoints is inside.
bool is_a_rigid_motion(const transform &tf)
{
    const rotation r = rigid_motion::rotation_matrix_from_transform(tf);

    return is_approx_equal(rotation(r.transpose() * r), rotation::Identity()) && is_approx_equal(r.determinant(), 1.0) &&
            is_approx_equal((tf.row(3) - Eigen::RowVector4d::UnitW()).cwiseAbs().maxCoeff(), 0.0);
}

}

expected<configuration, refusal> joint_straight_line(const configuration &start, const configuration &end, double s)
{
    if(start.size() != end.size())
        return unexpected(refusal::unsupported_input);

    return configuration(start + s * (end - start));
}

// Lynch & Park, Modern Robotics, eq. (9.6): a single constant screw axis carries the start frame
// onto the end frame, so the origin leaves the straight line between the two.
expected<transform, refusal> screw(const transform &start, const transform &end, double s)
{
    if(!is_a_rigid_motion(start) || !is_a_rigid_motion(end))
        return unexpected(refusal::degenerate);

    const expected<std::pair<screw_axis, double>, refusal> logged = rigid_motion::matrix_logarithm_se3(rigid_motion::inverse(start) * end);
    if(!logged)
        return unexpected(logged.error());

    return transform(start * rigid_motion::matrix_exponential_screw(logged->first, logged->second * s));
}

// Lynch & Park, Modern Robotics, eq. (9.8): the origin travels the straight line between the two
// positions while the rotation interpolates independently of it.
expected<transform, refusal> decoupled(const transform &start, const transform &end, double s)
{
    if(!is_a_rigid_motion(start) || !is_a_rigid_motion(end))
        return unexpected(refusal::degenerate);

    const rotation from_frame = rigid_motion::rotation_matrix_from_transform(start);

    const expected<std::pair<Eigen::Vector3d, double>, refusal> logged = rigid_motion::matrix_logarithm_so3(from_frame.transpose() * rigid_motion::rotation_matrix_from_transform(end));
    if(!logged)
        return unexpected(logged.error());

    const Eigen::Vector3d from = start.block<3, 1>(0, 3);
    const Eigen::Vector3d to   = end.block<3, 1>(0, 3);

    return transform(
            rigid_motion::transformation_matrix_from_rotation_position(from_frame * rigid_motion::matrix_exponential_so3(logged->first, logged->second * s), from + s * (to - from)));
}

}
