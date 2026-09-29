#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_LITERAL_EXPONENTIAL_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_LITERAL_EXPONENTIAL_H

#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/capabilities.h"

#include <Eigen/Core>

#include <cmath>

namespace praxis::fixture {

// Lynch & Park, Modern Robotics, eq. (3.88), as written for a unit angular part.
inline praxis::transform book_literal_exponential(const praxis::screw_axis &axis, double theta)
{
    const Eigen::Vector3d w = axis.head<3>();
    const Eigen::Vector3d v = axis.tail<3>();
    Eigen::Matrix3d skew;
    skew << 0.0, -w.z(), w.y(), w.z(), 0.0, -w.x(), -w.y(), w.x(), 0.0;
    const Eigen::Matrix3d squared = skew * skew;

    praxis::transform placed     = praxis::transform::Identity();
    placed.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() + std::sin(theta) * skew + (1.0 - std::cos(theta)) * squared;
    placed.block<3, 1>(0, 3)     = (Eigen::Matrix3d::Identity() * theta + (1.0 - std::cos(theta)) * skew + (theta - std::sin(theta)) * squared) * v;

    return placed;
}

inline praxis::rigid_motion::screw_ops exponentiating_literally()
{
    praxis::rigid_motion::screw_ops literal = praxis::rigid_motion::baseline().screw;
    literal.matrix_exponential_screw        = &book_literal_exponential;

    return literal;
}

}

#endif
