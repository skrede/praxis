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

// The bounds the screw path row is judged at: a rotation in radians and a distance in meters.
inline constexpr double screw_path_tolerance_radians = 1.0e-6;
inline constexpr double screw_path_tolerance_meters  = 1.0e-5;

// The bounds the decoupled path row is judged at: a rotation in radians and a distance in meters.
inline constexpr double decoupled_path_tolerance_radians = 1.0e-6;
inline constexpr double decoupled_path_tolerance_meters  = 1.0e-12;

// The dimensionless bound, element by element, at which the two path rows hold the rotation between
// their answers to orthonormality and its determinant to one.
inline constexpr double path_rotation_membership_tolerance = 1.0e-6;

// A driven configuration joined through drawn via points in radians, its rate in radians per second,
// and its acceleration as a fraction of the larger of one and either side's magnitude.
inline constexpr double via_point_run_tolerance = 1.0e-10;

// A chosen via point's time as a fraction of the larger of one and its magnitude, its position in
// radians and its velocity in radians per second.
inline constexpr double via_point_schedule_tolerance = 1.0e-8;

// The run the bound above was measured over: the shortest segment a run draws has no floor.
inline constexpr std::size_t via_point_schedule_measured_to_cases = 400000;

// A driven pose and both its twists: the rotation and the two angular parts in radians, per second
// and per second squared.
inline constexpr double driven_pose_tolerance_radians = 1.0e-3;

// The same sample's translation and the two linear parts, in meters, per second and per second
// squared.
inline constexpr double driven_pose_tolerance_meters = 1.0e-2;

const evaluation::capability_evaluations<path_ops> &path_evaluations();
const evaluation::capability_evaluations<trajectory_ops> &trajectory_evaluations();
const evaluation::capability_evaluations<time_scaling_ops> &time_scaling_evaluations();
const evaluation::capability_evaluations<pose_trajectory_ops> &pose_trajectory_evaluations();

}

#endif
