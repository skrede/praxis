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

// One cubic per segment per degree of freedom through each via point's position and velocity at its
// time (Lynch & Park, Modern Robotics, sec. 9.3, eq. (9.25)-(9.29)).
expected<std::unique_ptr<trajectory_generator>, refusal> joint_space_via_points(std::span<const via_point> via_points);

// The via points of the run through `positions`, prefixed by j0 where there is one: each segment the
// time its slowest joint needs at its velocity bound, interior velocities by Biagiotti & Melchiorri,
// sec. 2.1.4, eq. (2.3), rest at both ends, every time stretched by one factor so the cubic run's
// peak speed reaches the bound, and the velocities those at the stretched times.
expected<std::vector<via_point>, refusal> via_point_schedule(std::span<const configuration> positions, const configuration &j0, const configuration_limits &limits);

}

#endif
