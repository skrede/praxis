#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_PATH_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_PATH_H

#include "praxis/trajectory/path.h"

// Every declaration below matches a slot of path_ops by name and by signature, so composing the
// aggregate is a plain address-of.
namespace praxis::trajectory {

expected<configuration, refusal> joint_straight_line(const configuration &start, const configuration &end, double s);

expected<transform, refusal> screw(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &start, const transform &end, double s);
expected<transform, refusal> decoupled(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &start, const transform &end, double s);

}

#endif
