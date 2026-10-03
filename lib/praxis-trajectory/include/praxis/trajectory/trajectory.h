#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_TRAJECTORY_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_TRAJECTORY_H

#include "praxis/trajectory/types.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <span>
#include <memory>

namespace praxis::trajectory {

struct trajectory_sample
{
    configuration position;
    configuration velocity;
    configuration acceleration;
};

class trajectory_generator
{
public:
    trajectory_generator()                                        = default;
    trajectory_generator(const trajectory_generator &)            = delete;
    trajectory_generator(trajectory_generator &&)                 = delete;
    trajectory_generator &operator=(const trajectory_generator &) = delete;
    trajectory_generator &operator=(trajectory_generator &&)      = delete;
    virtual ~trajectory_generator()                               = default;

    // Sampling is by absolute time, so two calls at the same t give the same sample and calls may
    // arrive in any order. Owning a clock, abandoning a motion part way and replaying it at a
    // fraction of real speed are properties of whatever drives this and are expressed in the t it
    // passes; the end of the motion is t >= duration().
    virtual expected<trajectory_sample, refusal> sample(double t) const = 0;

    virtual double duration() const = 0;
};

}

namespace praxis::trajectory::inert {

expected<std::unique_ptr<trajectory_generator>, refusal> joint_space_waypoints(std::span<const configuration> waypoints, const configuration &j0, const configuration_limits &limits);

}

namespace praxis::trajectory {

// Declaration order is frozen: a designated initializer must name members in declaration order, so
// reordering a slot breaks every project that already composes this aggregate. Appending is safe.
// Only a via-point factory is here: its coefficients are solved once and sampled many times. A
// point-to-point motion is a path composed with a time scaling and needs no prepared object. The
// kinematic limits enter at construction, since the duration is derived from them.
//
// A single configuration is reached from j0 in a straight line at constant speed, and two are joined
// by one. Three or more are joined by one cubic per segment, Lynch & Park, Modern Robotics,
// sec. 9.3, eq. (9.26)-(9.29), at rest at both ends, with each interior velocity by Biagiotti &
// Melchiorri, Trajectory Planning for Automatic Machines and Robots, sec. 2.1.4, eq. (2.3): zero
// where the slopes on its two sides differ in sign, a zero slope having a sign of its own, and their
// mean otherwise. Each segment takes the time its slowest joint needs at its velocity bound, joints
// whose bound is not positive skipped, and every knot time of a cubic run is then stretched by one
// factor, the largest ratio of a joint's peak speed on any segment to its bound where that exceeds
// one. A run is judged by its duration, to 1e-12 relative and to 1e-12 s below one second, and by
// samples at evenly spaced times from 0 to the duration inclusive. At each, position and velocity
// are judged by their difference, and a degree of freedom's acceleration by its difference over
// the larger of one and the two sides' magnitudes; at the first and last sample that acceleration
// is accepted where the two sides agree or where either reports rest. Rows of differing width and
// an empty run are refused as unsupported_input, and so, in a run of three or more, are a row
// repeating the configuration before it and a coordinate that is not finite.
struct trajectory_ops
{
    expected<std::unique_ptr<trajectory_generator>, refusal> (*joint_space_waypoints)(std::span<const configuration> waypoints, const configuration &j0,
                                                                                      const configuration_limits &limits) = &inert::joint_space_waypoints;
};

}

#endif
