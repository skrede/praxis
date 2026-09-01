#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_MOTION_COMMANDS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_MOTION_COMMANDS_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/motion.h"

#include "praxis/trajectory/path.h"
#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/time_scaling.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/rigid_motion/types.h"

#include <memory>
#include <cstdint>
#include <optional>

namespace praxis::manipulator {

using task_space_path = expected<transform, refusal> (*)(const transform &start, const transform &end, double s);

enum class time_scaling_choice : std::uint8_t
{
    cubic,
    quintic,
    trapezoidal
};

// Both bound the path parameter: the rate is ds/dt and the rate of change is d2s/dt2. Neither
// bounds any joint.
struct path_parameter_bounds
{
    double max_rate;
    double max_rate_change;
};

// What the arm's limits leave the path parameter over a joint straight line, by the chain rule:
// dtheta_i/dt = delta_i s'(t), so s' is at most v_i / |delta_i| and s'' at most a_i / |delta_i| over
// every joint the motion moves. A motion of no extent bounds neither and answers finite stand-ins.
path_parameter_bounds derived_bounds(const joint_limits &bounds, const joint_vector &start, const joint_vector &target);

// A time scaling and the rule deriving a motion's duration from the arm's limits, carried together
// so that a caller composing a motion states neither. A trapezoidal choice holds the bounds a caller
// overrode, or holds none and takes what the motion's own extent gives.
class prepared_time_scaling
{
public:
    prepared_time_scaling(const trajectory::time_scaling_ops &injected_time_scaling, time_scaling_choice chosen);
    prepared_time_scaling(const trajectory::time_scaling_ops &injected_time_scaling, time_scaling_choice chosen, path_parameter_bounds held_to);

    // The same scaling and choice, holding bounds resolved once the motion's extent was known.
    prepared_time_scaling(const prepared_time_scaling &scaled, path_parameter_bounds held_to);

    // A trapezoidal profile is the bounds it was built from, so a value holding none has nothing to
    // sample against and refuses rather than inventing a pair.
    expected<trajectory::scaling_sample, refusal> sample(double t, double duration) const;

    // Never below the duration the trapezoidal profile reports for the same bounds, because the
    // bound baseline rescales that profile to what it is handed and rescaling only stretches.
    double duration(const joint_limits &bounds, const joint_vector &start, const joint_vector &target) const;

    time_scaling_choice chosen() const;

    std::optional<path_parameter_bounds> held_to() const;

private:
    time_scaling_choice m_chosen;
    trajectory::time_scaling_ops m_scaling;
    std::optional<path_parameter_bounds> m_bounds;
};

// A commanded motion is a path composed with a time scaling: Lynch & Park, Modern Robotics,
// ch. 9.2. Neither factory prepares anything -- both compose slots the caller supplies, and a
// composition that does not begin at the configuration the arm is in is refused.
expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> joint_space_motion(const trajectory::path_ops &injected_path, const prepared_time_scaling &scaled,
                                                                                        const joint_limits &bounds, const joint_vector &start, const joint_vector &target);

// The task-space path is resolved to a configuration through the motion capability at every sample,
// always from the same seed, so that sampling stays a pure function of the path parameter.
expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> task_space_motion(const motion_ops &injected_motion, const prepared_time_scaling &scaled, const kinematics &solver,
                                                                                       const joint_limits &bounds, const joint_vector &start, const transform &start_pose,
                                                                                       const transform &end_pose, task_space_path shape);

}

#endif
