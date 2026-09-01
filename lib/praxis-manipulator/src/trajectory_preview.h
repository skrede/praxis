#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_TRAJECTORY_PREVIEW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_TRAJECTORY_PREVIEW_H

#include "praxis/manipulator/scene_robot.h"
#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/motion_commands.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/trajectory/trajectory.h"

#include <span>
#include <vector>

namespace praxis::manipulator {

// Samples a motion whole, at the density the drawn preview is held to, without moving the arm: every
// tool pose is taken at the configuration the same sample carries. A motion that refuses at any
// instant is refused entirely, so a published run always spans its motion. An empty set of scalings
// is a motion whose timing is its own polynomial and carries no curve; otherwise one curve is
// answered per scaling, in the order given, sampled at the samples' own times.
expected<preview_run, refusal> sampled_preview(const trajectory::trajectory_generator &motion, const scene_robot &arm, std::span<const prepared_time_scaling> scalings);

// The path parameter the samples given realize: the distance travelled along the configuration-space
// path they trace, normalized to the whole, with its two derivatives with respect to time. It is
// derived from the samples rather than sampled a second time, so it is answered at the samples' own
// times and one entry per sample. Fewer than two samples, and samples standing at one configuration
// throughout, answer nothing rather than a quotient by a distance of zero.
std::vector<trajectory::scaling_sample> realized_parameter(std::span<const preview_sample> taken);

}

#endif
