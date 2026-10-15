#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_SUBSTITUTED_RIGID_MOTION_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_SUBSTITUTED_RIGID_MOTION_H

#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/evaluation/tolerance.h"

#include <Eigen/Core>

// Every function below answers differently from the reference it stands in for, at every input the
// suite passes it, and the predicates at the foot of the file are what hold that true.
namespace praxis::fixture {

inline transform marked_inverse(const transform &)
{
    return rigid_motion::transformation_matrix_from_position(Eigen::Vector3d(5.0, 6.0, 7.0));
}

inline transform turned_further(const screw_axis &s, double theta)
{
    return rigid_motion::matrix_exponential_screw(s, theta + 1.0);
}

inline expected<screw_axis, refusal> reversed_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{
    return rigid_motion::screw_axis_from_point_direction_pitch(q, -s, h);
}

inline transform displaced_pose(const rotation &r, const Eigen::Vector3d &p)
{
    return rigid_motion::transformation_matrix_from_rotation_position(r, p + Eigen::Vector3d(1.0, 1.0, 1.0));
}

// Scaled rather than rotated: a scaled rotation is no longer orthonormal, so a caller that only
// tests the answer for rigidity sees the substitution too.
inline rotation scaled_rotation(const transform &tf)
{
    return rotation(2.0 * rigid_motion::rotation_matrix_from_transform(tf));
}

inline expected<adjoint, refusal> scaled_adjoint(const rotation &r, const Eigen::Vector3d &p)
{
    const expected<adjoint, refusal> reference = rigid_motion::adjoint_matrix_from_rotation_position(r, p);
    if(!reference)
        return reference;

    return adjoint(2.0 * *reference);
}

// A path swept under these three still stands at its start pose at s = 0 and stays in the plane the
// start and end poses share, so a planar arm still resolves every pose along it.
inline rotation turned_half_again(const Eigen::Vector3d &w, double theta)
{
    return rigid_motion::matrix_exponential_so3(w, 1.5 * theta);
}

inline transform screwed_half_again(const screw_axis &s, double theta)
{
    return rigid_motion::matrix_exponential_screw(s, 1.5 * theta);
}

inline transform inverse_then_turned(const transform &tf)
{
    return rigid_motion::transformation_matrix_from_rotation(rigid_motion::rotate_z(0.2)) * rigid_motion::inverse(tf);
}

inline rigid_motion::capabilities only_the_inverse()
{
    rigid_motion::capabilities spatial = rigid_motion::baseline();
    spatial.frame.inverse              = &marked_inverse;

    return spatial;
}

inline rigid_motion::capabilities only_the_exponential()
{
    rigid_motion::capabilities spatial     = rigid_motion::baseline();
    spatial.screw.matrix_exponential_screw = &turned_further;

    return spatial;
}

inline rigid_motion::capabilities substituted_everywhere()
{
    rigid_motion::capabilities spatial                         = rigid_motion::baseline();
    spatial.frame.rotation_matrix_from_transform               = &scaled_rotation;
    spatial.frame.transformation_matrix_from_rotation_position = &displaced_pose;
    spatial.frame.inverse                                      = &marked_inverse;
    spatial.screw.adjoint_matrix_from_rotation_position        = &scaled_adjoint;
    spatial.screw.screw_axis_from_point_direction_pitch        = &reversed_axis;
    spatial.screw.matrix_exponential_screw                     = &turned_further;

    return spatial;
}

inline rigid_motion::capabilities substituted_along_a_path()
{
    rigid_motion::capabilities spatial     = rigid_motion::baseline();
    spatial.frame.inverse                  = &inverse_then_turned;
    spatial.screw.matrix_exponential_so3   = &turned_half_again;
    spatial.screw.matrix_exponential_screw = &screwed_half_again;

    return spatial;
}

inline bool every_substituted_slot_differs(const rigid_motion::capabilities &reference, const rigid_motion::capabilities &perturbed)
{
    const Eigen::Vector3d point{0.2, 0.3, 0.0};
    const Eigen::Vector3d direction = Eigen::Vector3d::UnitZ();
    const rotation spun             = rigid_motion::rotate_z(0.3);
    const transform pose            = rigid_motion::transformation_matrix_from_rotation_position(spun, point);
    const screw_axis axis           = *reference.screw.screw_axis_from_point_direction_pitch(point, direction, 0.0);

    return !is_approx_equal(perturbed.frame.inverse(pose), reference.frame.inverse(pose)) &&
            !perturbed.frame.rotation_matrix_from_transform(pose).isApprox(reference.frame.rotation_matrix_from_transform(pose)) &&
            !is_approx_equal(perturbed.frame.transformation_matrix_from_rotation_position(spun, point), pose) &&
            !perturbed.screw.adjoint_matrix_from_rotation_position(spun, point)->isApprox(*reference.screw.adjoint_matrix_from_rotation_position(spun, point)) &&
            !perturbed.screw.screw_axis_from_point_direction_pitch(point, direction, 0.0)->isApprox(axis) &&
            !perturbed.screw.matrix_exponential_screw(axis, 0.721).isApprox(reference.screw.matrix_exponential_screw(axis, 0.721));
}

inline bool every_path_substitution_differs(const rigid_motion::capabilities &reference, const rigid_motion::capabilities &perturbed)
{
    const Eigen::Vector3d direction = Eigen::Vector3d::UnitZ();
    const transform pose            = rigid_motion::transformation_matrix_from_rotation_position(rigid_motion::rotate_z(0.3), Eigen::Vector3d(0.2, 0.3, 0.0));
    const screw_axis axis           = *reference.screw.screw_axis_from_point_direction_pitch(Eigen::Vector3d(0.2, 0.3, 0.0), direction, 0.0);

    return !is_approx_equal(perturbed.frame.inverse(pose), reference.frame.inverse(pose)) &&
            !perturbed.screw.matrix_exponential_so3(direction, 0.721).isApprox(reference.screw.matrix_exponential_so3(direction, 0.721)) &&
            !perturbed.screw.matrix_exponential_screw(axis, 0.721).isApprox(reference.screw.matrix_exponential_screw(axis, 0.721));
}

}

#endif
