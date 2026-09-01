#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_TASK_TRAJECTORY_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_TASK_TRAJECTORY_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/kinematics.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/rigid_motion/types.h"

#include "praxis/trajectory/trajectory.h"

#include <span>
#include <memory>

namespace praxis::manipulator::inert {

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> task_space_waypoints(const kinematics &solver, std::span<const transform> waypoints, const joint_vector &j0,
                                                                                          const joint_limits &limits);

}

namespace praxis::manipulator {

// Declaration order is frozen: a designated initializer must name members in declaration order, so
// reordering a slot breaks every project that already composes this aggregate. Appending is safe.
// Only the via-point factory is here: its coefficients are solved once and sampled many times. A
// point-to-point motion is a path composed with a time scaling and needs no prepared object. The
// kinematic limits enter at construction, since the duration is derived from them.
struct task_trajectory_ops
{
    expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> (*task_space_waypoints)(const kinematics &solver, std::span<const transform> waypoints, const joint_vector &j0,
                                                                                                 const joint_limits &limits) = &inert::task_space_waypoints;
};

}

#endif
