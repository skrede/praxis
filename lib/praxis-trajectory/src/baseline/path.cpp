#include "praxis/trajectory/baseline/path.h"

#include "praxis/evaluation/tolerance.h"

namespace praxis::trajectory {

namespace {

// Membership of SE(3): a finite matrix, an orthonormal rotation of unit determinant over an affine bottom row. Both
// task-space paths premultiply by the start frame, so an endpoint outside the set produces an
// interpolated pose outside it even where the relative motion between the two endpoints is inside.
bool is_a_rigid_motion(const rigid_motion::frame_ops &frames, const transform &tf)
{
    const rotation r = frames.rotation_matrix_from_transform(tf);

    return tf.allFinite() && is_approx_equal(rotation(r.transpose() * r), rotation::Identity()) && is_approx_equal(r.determinant(), 1.0) &&
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
expected<transform, refusal> screw(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &start, const transform &end, double s)
{
    if(!is_a_rigid_motion(frames, start) || !is_a_rigid_motion(frames, end))
        return unexpected(refusal::degenerate);

    const expected<std::pair<screw_axis, double>, refusal> logged = screw.matrix_logarithm_se3(transform(frames.inverse(start) * end));
    if(!logged)
        return unexpected(logged.error());

    return transform(start * screw.matrix_exponential_screw(logged->first, logged->second * s));
}

// Lynch & Park, Modern Robotics, eq. (9.8): the origin travels the straight line between the two
// positions while the rotation interpolates independently of it.
expected<transform, refusal> decoupled(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &start, const transform &end, double s)
{
    if(!is_a_rigid_motion(frames, start) || !is_a_rigid_motion(frames, end))
        return unexpected(refusal::degenerate);

    const rotation from_frame = frames.rotation_matrix_from_transform(start);

    const expected<std::pair<Eigen::Vector3d, double>, refusal> logged = screw.matrix_logarithm_so3(from_frame.transpose() * frames.rotation_matrix_from_transform(end));
    if(!logged)
        return unexpected(logged.error());

    const Eigen::Vector3d from = start.block<3, 1>(0, 3);
    const Eigen::Vector3d to   = end.block<3, 1>(0, 3);

    return transform(frames.transformation_matrix_from_rotation_position(from_frame * screw.matrix_exponential_so3(logged->first, logged->second * s), from + s * (to - from)));
}

}
