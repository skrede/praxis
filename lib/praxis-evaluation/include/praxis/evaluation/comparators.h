#ifndef HPP_GUARD_PRAXIS_EVALUATION_COMPARATORS_H
#define HPP_GUARD_PRAXIS_EVALUATION_COMPARATORS_H

#include "praxis/evaluation/residual.h"

#include <Eigen/Core>

namespace praxis::evaluation {

// The greatest absolute difference between corresponding elements. One dynamic reference serves
// every shape a comparison is written over, and the answer does not depend on which side is first.
residual element_wise_residual(const Eigen::Ref<const Eigen::MatrixXd> &first, const Eigen::Ref<const Eigen::MatrixXd> &second);

// The geodesic distance on SO(3): the rotation carried by `first` transposed times `second`,
// reported in radians and never negative.
residual geodesic_residual(const Eigen::Matrix3d &first, const Eigen::Matrix3d &second);

// The two halves of a pose discrepancy, carried apart and never summed: the geodesic angle between
// the two rotation blocks in radians as the magnitude, and the distance between the two origins in
// metres as the linear half. Each half is judged against its own tolerance.
residual pose_residual(const Eigen::Matrix4d &first, const Eigen::Matrix4d &second);

// The distance between two axes read up to their sign, so an axis and its exact negation answer
// zero and an axis differing by anything else does not.
residual axis_up_to_sign_residual(const Eigen::Vector<double, 6> &first, const Eigen::Vector<double, 6> &second);

// A logarithm is not unique, so neither comparator below compares the axes or the angles it is
// given. Each exponentiates both answers and compares the two group elements, which is what the
// caller relies on: a branch differing by a full turn, or an axis-sign flip that names the same
// element, agrees, while an answer naming a different element does not.
residual log_up_to_branch_rotation_residual(const Eigen::Vector3d &first_axis, double first_theta_radians, const Eigen::Vector3d &second_axis, double second_theta_radians);

residual log_up_to_branch_pose_residual(const Eigen::Vector<double, 6> &first_axis, double first_theta_radians, const Eigen::Vector<double, 6> &second_axis,
                                        double second_theta_radians);

}

#endif
