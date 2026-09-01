#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_TYPES_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_TYPES_H

#include <Eigen/Core>

namespace praxis {

using transform  = Eigen::Matrix4d;
using rotation   = Eigen::Matrix3d;
using matrix3    = Eigen::Matrix3d;
using matrix4    = Eigen::Matrix4d;
using twist      = Eigen::Vector<double, 6>;
using screw_axis = Eigen::Vector<double, 6>;
using adjoint    = Eigen::Matrix<double, 6, 6>;

// The norm at or below which a screw's angular part names no axis at all. Both the line a screw
// fixes and its pitch are read by dividing the linear part by this norm, so every reader of a screw
// has to draw the boundary in the same place or two drawings of one screw disagree about whether it
// has one.
inline constexpr double angular_epsilon = 1.0e-12;

}

#endif
