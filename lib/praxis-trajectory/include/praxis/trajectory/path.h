#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_PATH_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_PATH_H

#include "praxis/trajectory/types.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/rigid_motion/types.h"

namespace praxis::trajectory::inert {

expected<configuration, refusal> joint_straight_line(const configuration &start, const configuration &end, double s);

expected<transform, refusal> screw(const transform &start, const transform &end, double s);
expected<transform, refusal> decoupled(const transform &start, const transform &end, double s);

}

namespace praxis::trajectory {

// Declaration order is frozen: a designated initializer must name members in declaration order, so
// reordering a slot breaks every project that already composes this aggregate. Appending is safe.
// A path is a pure function of the path parameter s in [0,1] and holds no state: Lynch & Park,
// Modern Robotics, eq. (9.3), (9.6), (9.7) and (9.8). The two task-space paths differ in whether
// the rotation is coupled to the translation -- the screw path holds a constant screw axis and its
// origin leaves the straight line, the decoupled path moves the origin along the straight line at
// constant angular velocity.
struct path_ops
{
    expected<configuration, refusal> (*joint_straight_line)(const configuration &start, const configuration &end, double s) = &inert::joint_straight_line;

    expected<transform, refusal> (*screw)(const transform &start, const transform &end, double s)     = &inert::screw;
    expected<transform, refusal> (*decoupled)(const transform &start, const transform &end, double s) = &inert::decoupled;
};

}

#endif
