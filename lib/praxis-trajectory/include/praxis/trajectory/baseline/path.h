#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_PATH_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_PATH_H

#include "praxis/trajectory/path.h"

// Every declaration below matches a slot of path_ops by name and by signature, so composing the
// aggregate is a plain address-of.
namespace praxis::trajectory {

expected<configuration, refusal> joint_straight_line(const configuration &start, const configuration &end, double s);

expected<transform, refusal> screw(const transform &start, const transform &end, double s);
expected<transform, refusal> decoupled(const transform &start, const transform &end, double s);

}

#endif
