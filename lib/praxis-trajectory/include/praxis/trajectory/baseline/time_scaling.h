#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_TIME_SCALING_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_TIME_SCALING_H

#include "praxis/trajectory/time_scaling.h"

// Every declaration below matches a slot of time_scaling_ops by name and by signature, so composing
// the aggregate is a plain address-of.
namespace praxis::trajectory {

expected<scaling_sample, refusal> cubic(double t, double duration);
expected<scaling_sample, refusal> quintic(double t, double duration);
expected<scaling_sample, refusal> trapezoidal_from_velocity_acceleration(double t, double velocity, double acceleration);
expected<scaling_sample, refusal> trapezoidal_from_velocity_duration(double t, double duration, double velocity);
expected<scaling_sample, refusal> trapezoidal_from_acceleration_duration(double t, double duration, double acceleration);
expected<scaling_sample, refusal> trapezoidal_within_bounds(double t, double duration, double max_velocity, double max_acceleration);

}

#endif
