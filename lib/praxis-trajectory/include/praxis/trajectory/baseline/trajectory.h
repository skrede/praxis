#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_TRAJECTORY_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_TRAJECTORY_H

#include "praxis/trajectory/trajectory.h"

namespace praxis::trajectory {

// One or two configurations are joined by a straight line traversed at constant speed (Lynch & Park,
// Modern Robotics, sec. 9.2). Three or more are traversed as one cubic per segment through every via
// point (sec. 9.3, eq. (9.26)-(9.29)), with interior velocities by Biagiotti & Melchiorri, Trajectory
// Planning for Automatic Machines and Robots, sec. 2.1.4, eq. (2.3), and zero velocity at both ends;
// velocity is continuous at every via point and acceleration steps there.
expected<std::unique_ptr<trajectory_generator>, refusal> joint_space_waypoints(std::span<const configuration> waypoints, const configuration &j0, const configuration_limits &limits);

}

#endif
