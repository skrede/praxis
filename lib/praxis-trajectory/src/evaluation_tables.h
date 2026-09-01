#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_EVALUATION_TABLES_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_EVALUATION_TABLES_H

#include "praxis/trajectory/path.h"
#include "praxis/trajectory/slots.h"
#include "praxis/trajectory/time_scaling.h"

#include "praxis/evaluation/slot_evaluation.h"

#include <cstddef>

namespace praxis::trajectory {

// The path parameter and both its derivatives, in the parameter's own units and per second and per
// second squared of it.
inline constexpr double scaling_sample_tolerance = 1.0e-12;

// The same sample, where the scaling carrying it is a quintic.
inline constexpr double quintic_scaling_tolerance = 1.0e-11;

// A driven configuration and both its derivatives, in radians and per second and per second squared.
inline constexpr double driven_configuration_tolerance = 1.0e-4;

// The run the bound above was measured over. It is not known to hold over a longer one: the shortest
// segment a run draws has no floor, and the residual carries the inverse square of its duration.
inline constexpr std::size_t driven_configuration_measured_to_cases = 1000;

// A driven pose and both its twists: the rotation and the two angular parts in radians, per second
// and per second squared.
inline constexpr double driven_pose_tolerance_radians = 1.0e-3;

// The same sample's translation and the two linear parts, in metres, per second and per second
// squared.
inline constexpr double driven_pose_tolerance_metres = 1.0e-2;

const evaluation::capability_evaluations<path_ops> &path_evaluations();
const evaluation::capability_evaluations<trajectory_ops> &trajectory_evaluations();
const evaluation::capability_evaluations<time_scaling_ops> &time_scaling_evaluations();
const evaluation::capability_evaluations<pose_trajectory_ops> &pose_trajectory_evaluations();

}

#endif
