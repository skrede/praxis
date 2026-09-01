#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_THREE_LINK_ARM_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_THREE_LINK_ARM_H

#include "praxis/manipulator/screw_chain.h"

#include <vector>

namespace praxis::fixture {

using namespace manipulator;

inline constexpr double upper_arm = 0.5;
inline constexpr double forearm   = 0.4;
inline constexpr double wrist     = 0.3;

// A reference carries no solver parameters, so every solve below stops at the contract's default
// task-space epsilon of 1e-5. A round trip through one is asserted an order above that.
inline constexpr double solved_tolerance = 1.0e-4;

// Lynch & Park, Modern Robotics, eq. (3.24): a revolute screw is (w, -w x q) for a point q on its
// axis. All three axes are the world z through a point on the x axis.
inline screw_axis about_z(double x)
{
    screw_axis axis;
    axis << 0.0, 0.0, 1.0, 0.0, -x, 0.0;

    return axis;
}

// Three revolute joints in the xy plane: position and orientation within that plane are reachable
// independently of each other, which two links are not enough for.
inline screw_chain three_link_arm()
{
    transform home = transform::Identity();
    home(0, 3)     = upper_arm + forearm + wrist;

    joint_limits bounds{};
    bounds.velocity       = joint_vector::Constant(3, 1.0);
    bounds.acceleration   = joint_vector::Constant(3, 2.0);
    bounds.lower_position = joint_vector::Constant(3, -3.0);
    bounds.upper_position = joint_vector::Constant(3, 3.0);

    return screw_chain(home, {about_z(0.0), about_z(upper_arm), about_z(upper_arm + forearm)}, bounds);
}

inline joint_vector arm_configuration(double first, double second, double third)
{
    joint_vector q(3);
    q << first, second, third;

    return q;
}

// Inside the workspace, away from the folded and outstretched singularities, and with a tool
// orientation that is not the identity.
inline joint_vector posed_arm()
{
    return arm_configuration(0.4, -1.2, 0.5);
}

}

#endif
