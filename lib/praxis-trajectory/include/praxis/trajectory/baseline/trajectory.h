#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_TRAJECTORY_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_TRAJECTORY_H

#include "praxis/trajectory/trajectory.h"

namespace praxis::trajectory {

// One or two configurations are joined by a straight line traversed at constant speed; three or more
// are traversed as one motion whose velocity is continuous at every via point and zero at both ends.
// Lynch & Park, Modern Robotics, sec. 9.2 and 9.3.
expected<std::unique_ptr<trajectory_generator>, refusal> joint_space_waypoints(std::span<const configuration> waypoints, const configuration &j0, const configuration_limits &limits);

}

#endif
