#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_POSE_TRAJECTORY_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_POSE_TRAJECTORY_H

#include "praxis/trajectory/pose_trajectory.h"

namespace praxis::trajectory {

// One task-space path carries the seed onto the waypoint -- the decoupled path of Lynch & Park,
// Modern Robotics, eq. (9.6), or the constant screw axis of eq. (9.8) -- traversed under the quintic
// time scaling of this same extension, eq. (9.10). The reported twist and its derivative are the
// chain rule applied to that composition, so both derivatives of the scaling reach the sample. A run
// of three or more poses is one such traversal per consecutive pair, concatenated in time: it stands
// still at every pose between its two ends, and the twists it reports at an instant are in the start
// frame of the pair that instant falls in.
expected<std::unique_ptr<pose_trajectory_generator>, refusal> decoupled_pose_waypoints(std::span<const transform> waypoints, const transform &seed, double max_linear_speed,
                                                                                       double max_angular_speed);
expected<std::unique_ptr<pose_trajectory_generator>, refusal> screw_pose_waypoints(std::span<const transform> waypoints, const transform &seed, double max_linear_speed,
                                                                                   double max_angular_speed);

}

#endif
