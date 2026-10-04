#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_BASELINE_GROUP_MEMBERSHIP_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_BASELINE_GROUP_MEMBERSHIP_H

#include "praxis/rigid_motion/types.h"

#include "praxis/evaluation/tolerance.h"

namespace praxis::rigid_motion {

inline bool is_a_rotation(const rotation &r)
{
    return is_approx_equal(rotation(r.transpose() * r), rotation::Identity()) && is_approx_equal(r.determinant(), 1.0);
}

inline bool is_a_rigid_motion(const transform &tf)
{
    const Eigen::VectorXd bottom = tf.row(3).transpose();

    return is_a_rotation(tf.block<3, 3>(0, 0)) && is_approx_equal(bottom, Eigen::VectorXd(Eigen::Vector4d::UnitW()));
}

}

#endif
