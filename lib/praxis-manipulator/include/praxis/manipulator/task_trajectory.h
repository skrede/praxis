#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_TASK_TRAJECTORY_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_TASK_TRAJECTORY_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/motion.h"
#include "praxis/manipulator/kinematics.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/rigid_motion/types.h"

#include "praxis/trajectory/trajectory.h"

#include <span>
#include <memory>

namespace praxis::manipulator::inert {

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> task_space_waypoints(const trajectory::trajectory_ops &trajectory, const motion_ops &motion,
                                                                                          const kinematics &solver, std::span<const transform> waypoints, const joint_vector &j0,
                                                                                          const joint_limits &limits);

}

namespace praxis::manipulator {

// Only the via-point factory is here: its coefficients are solved once and sampled many times. A
// point-to-point motion is a path composed with a time scaling and needs no prepared object. The
// kinematic limits enter at construction, since the duration is derived from them.
// Each pose resolves through motion.task_space_pose, seeded by the configuration the one before
// resolved to. The configurations are joined by the step and the via-point slot of trajectory, and
// the prepared motion is judged by that slot's contract.
struct task_trajectory_ops
{
    expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> (*task_space_waypoints)(const trajectory::trajectory_ops &trajectory, const motion_ops &motion,
                                                                                                 const kinematics &solver, std::span<const transform> waypoints, const joint_vector &j0,
                                                                                                 const joint_limits &limits) = &inert::task_space_waypoints;
};

}

#endif
