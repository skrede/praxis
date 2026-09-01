#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_TASK_TRAJECTORY_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_TASK_TRAJECTORY_H

#include "praxis/manipulator/task_trajectory.h"

namespace praxis::manipulator {

// Each pose is resolved from the configuration the previous one resolved to, so a chain of
// waypoints stays on one inverse-kinematics branch as far as the solver allows.
expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> task_space_waypoints(const kinematics &solver, std::span<const transform> waypoints, const joint_vector &j0,
                                                                                          const joint_limits &limits);

}

#endif
