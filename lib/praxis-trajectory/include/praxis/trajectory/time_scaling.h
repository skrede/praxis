#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_TIME_SCALING_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_TIME_SCALING_H

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

namespace praxis::trajectory {

// The path parameter and its first and second derivatives with respect to time. Both derivatives
// are carried because a trajectory's velocity and acceleration follow from them by the chain rule:
// Lynch & Park, Modern Robotics, eq. (9.1) and (9.2). A sample carrying only s makes those
// identities inexpressible.
struct scaling_sample
{
    double s;
    double ds;
    double dds;
};

}

namespace praxis::trajectory::inert {

expected<scaling_sample, refusal> cubic(double t, double duration);
expected<scaling_sample, refusal> quintic(double t, double duration);
expected<scaling_sample, refusal> trapezoidal(double t, double duration, double max_velocity, double max_acceleration);

}

namespace praxis::trajectory {

// Declaration order is frozen: a designated initializer must name members in declaration order, so
// reordering a slot breaks every project that already composes this aggregate. Appending is safe.
// A time scaling is scalar mathematics: this header names no configuration, no pose and no solver.
//
// A scaling is judged over 0 <= t <= duration, and what it reports outside that interval is
// not judged. Where the second derivative steps -- at both ends, and at a trapezoid's ramp and
// coast boundaries -- either neighboring value is accepted; s and its first derivative are judged
// there as everywhere. Lynch & Park, Modern Robotics, sec. 9.2.2.
// The trapezoid holds max_acceleration and solves its cruise rate so that s reaches 1 at the
// duration given (sec. 9.2.2.2, choosing a and T). A duration shorter than the shortest its two
// bounds allow, a duration that is not finite and positive, or a bound that is not finite and
// positive is refused as unsupported_input, and a refusal carries no sample.
struct time_scaling_ops
{
    expected<scaling_sample, refusal> (*cubic)(double t, double duration)   = &inert::cubic;
    expected<scaling_sample, refusal> (*quintic)(double t, double duration) = &inert::quintic;

    expected<scaling_sample, refusal> (*trapezoidal)(double t, double duration, double max_velocity, double max_acceleration) = &inert::trapezoidal;
};

}

#endif
