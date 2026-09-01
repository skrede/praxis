#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_SWEPT_POSE_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_SWEPT_POSE_H

#include "praxis/trajectory/pose_trajectory.h"

namespace praxis::trajectory {

// The shape of the task-space paths of this extension, one of which a pose motion is swept along.
using task_space_path = expected<transform, refusal> (*)(const transform &start, const transform &end, double s);

// One path traversed under the quintic scaling, whose rate is zero at both ends, so the motion stands
// still at each of the two poses. Refuses a turn with no logarithm and a pair traversing in no time.
expected<std::unique_ptr<pose_trajectory_generator>, refusal> swept_pose(task_space_path along, const transform &from, const transform &to, double max_linear_speed,
                                                                         double max_angular_speed);

}

#endif
