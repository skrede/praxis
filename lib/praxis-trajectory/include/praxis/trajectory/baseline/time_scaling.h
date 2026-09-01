#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_TIME_SCALING_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_TIME_SCALING_H

#include "praxis/trajectory/time_scaling.h"

// Every declaration below matches a slot of time_scaling_ops by name and by signature, so composing
// the aggregate is a plain address-of.
namespace praxis::trajectory {

expected<scaling_sample, refusal> cubic(double t, double duration);
expected<scaling_sample, refusal> quintic(double t, double duration);
expected<scaling_sample, refusal> trapezoidal(double t, double duration, double max_velocity, double max_acceleration);

}

#endif
