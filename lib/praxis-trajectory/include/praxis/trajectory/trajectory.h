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
struct trajectory_ops
{
    expected<std::unique_ptr<trajectory_generator>, refusal> (*joint_space_waypoints)(std::span<const configuration> waypoints, const configuration &j0,
                                                                                      const configuration_limits &limits) = &inert::joint_space_waypoints;
};

}

#endif
