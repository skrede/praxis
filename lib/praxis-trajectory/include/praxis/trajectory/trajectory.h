#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_TRAJECTORY_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_TRAJECTORY_H

#include "praxis/trajectory/types.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <span>
#include <memory>
#include <vector>

namespace praxis::trajectory {

struct trajectory_sample
{
    configuration position;
    configuration velocity;
    configuration acceleration;
};

// A configuration the run stands at, at a time in seconds from its start, moving at a velocity.
struct via_point
{
    double time;
    configuration position;
    configuration velocity;
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

expected<std::unique_ptr<trajectory_generator>, refusal> joint_space_via_points(std::span<const via_point> via_points);
expected<std::vector<via_point>, refusal> via_point_schedule(std::span<const configuration> positions, const configuration &j0, const configuration_limits &limits);

}

namespace praxis::trajectory {

// Declaration order is frozen: a designated initializer must name members in declaration order, so
// reordering a slot breaks every project that already composes this aggregate. Appending is safe.
// Only the via-point factory and the step choosing its via points are here: a run's coefficients are
// solved once and sampled many times. A point-to-point motion is a path composed with a time scaling
// and needs no prepared object. The kinematic limits enter only the step, where a duration is derived
// from them.
//
// joint_space_via_points joins every pair of consecutive via points by one cubic per degree of
// freedom matching the position and velocity of both at their times, Lynch & Park, Modern Robotics,
// sec. 9.3, eq. (9.25)-(9.29). A run is judged by its duration, the last via point's time, to 1e-12
// relative and to 1e-12 s below one second, and by samples at evenly spaced times from 0 to the
// duration inclusive. At each, position and velocity are judged by their difference, and a degree of
// freedom's acceleration by its difference over the larger of one and the two sides' magnitudes; at
// the first and last sample that acceleration is accepted where the two sides agree or where either
// reports rest. A sample carrying a value that is not finite, on either side, agrees with nothing. A
// run handed for judgment holds at least two via points, starts at time 0, its times strictly
// increase, every position and velocity has one width and every value is finite; nothing outside
// that is judged, and a reference refuses it as unsupported_input.
//
// via_point_schedule chooses those via points from configurations. One configuration is joined to it
// from j0 and two are joined to each other. Three or more are at rest at both ends, with each interior
// velocity by Biagiotti & Melchiorri, Trajectory Planning for Automatic Machines and Robots,
// sec. 2.1.4, eq. (2.3): zero where the slopes on its two sides differ in sign, a zero slope having a
// sign of its own, and their mean otherwise. Each segment takes the time its slowest joint needs at
// its velocity bound, joints whose bound is not positive skipped, and every time is then stretched by
// one factor, the largest ratio of a joint's peak speed on any segment of the cubic run to its bound
// where that exceeds one, the velocities being those at the stretched times. A run of one or two is
// therefore at rest at both via points, over the slowest joint's traversal stretched until the
// cubic's peak speed, 1.5 times its average, reaches the bound (sec. 9.2.2.1). A run of one or two
// that does not move, an empty run, rows of differing width, a coordinate that is not finite, and in
// a run of three or more a row repeating the one before it are refused as unsupported_input. A
// schedule is judged by its length and element by element, each time against the larger of one and
// its magnitude and each position and velocity by its difference.
struct trajectory_ops
{
    expected<std::unique_ptr<trajectory_generator>, refusal> (*joint_space_via_points)(std::span<const via_point> via_points) = &inert::joint_space_via_points;
    expected<std::vector<via_point>, refusal> (*via_point_schedule)(std::span<const configuration> positions, const configuration &j0,
                                                                    const configuration_limits &limits)                       = &inert::via_point_schedule;
};

}

#endif
