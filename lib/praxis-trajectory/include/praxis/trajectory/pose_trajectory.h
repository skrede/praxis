#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_POSE_TRAJECTORY_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_POSE_TRAJECTORY_H

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/rigid_motion/types.h"

#include <span>
#include <memory>

namespace praxis::trajectory {

struct pose_sample
{
    transform position;
    twist velocity;
    twist acceleration;
};

class pose_trajectory_generator
{
public:
    pose_trajectory_generator()                                             = default;
    pose_trajectory_generator(const pose_trajectory_generator &)            = delete;
    pose_trajectory_generator(pose_trajectory_generator &&)                 = delete;
    pose_trajectory_generator &operator=(const pose_trajectory_generator &) = delete;
    pose_trajectory_generator &operator=(pose_trajectory_generator &&)      = delete;
    virtual ~pose_trajectory_generator()                                    = default;

    // Sampling is by absolute time, so two calls at the same t give the same sample and calls may
    // arrive in any order. Owning a clock, abandoning a motion part way and replaying it at a
    // fraction of real speed are properties of whatever drives this and are expressed in the t it
    // passes; the end of the motion is t >= duration().
    virtual expected<pose_sample, refusal> sample(double t) const = 0;

    virtual double duration() const = 0;
};

}

namespace praxis::trajectory::inert {

expected<std::unique_ptr<pose_trajectory_generator>, refusal> decoupled_pose_waypoints(std::span<const transform> waypoints, const transform &seed, double max_linear_speed,
                                                                                       double max_angular_speed);
expected<std::unique_ptr<pose_trajectory_generator>, refusal> screw_pose_waypoints(std::span<const transform> waypoints, const transform &seed, double max_linear_speed,
                                                                                   double max_angular_speed);

}

namespace praxis::trajectory {

// Declaration order is frozen: a designated initializer must name members in declaration order, so
// reordering a slot breaks every project that already composes this aggregate. Appending is safe.
// The generator's duration() is derived from the speed bounds passed here, and the two slots differ
// only in which of the task-space paths of this same extension the motion traverses.
struct pose_trajectory_ops
{
    expected<std::unique_ptr<pose_trajectory_generator>, refusal> (*decoupled_pose_waypoints)(std::span<const transform> waypoints, const transform &seed, double max_linear_speed,
                                                                                              double max_angular_speed) = &inert::decoupled_pose_waypoints;
    expected<std::unique_ptr<pose_trajectory_generator>, refusal> (*screw_pose_waypoints)(std::span<const transform> waypoints, const transform &seed, double max_linear_speed,
                                                                                          double max_angular_speed)     = &inert::screw_pose_waypoints;
};

}

#endif
