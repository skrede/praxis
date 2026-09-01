#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_STRAIGHT_LINE_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_STRAIGHT_LINE_H

#include "praxis/trajectory/trajectory.h"

namespace praxis::trajectory {

// The time the slowest degree of freedom needs to cross its own span at its velocity bound, which is
// the duration rule every segment of a configuration-space motion is apportioned by.
double traversal_time(const configuration &from, const configuration &to, const configuration &velocity);

expected<std::unique_ptr<trajectory_generator>, refusal> straight_line(const configuration &from, const configuration &to, const configuration_limits &limits);

}

#endif
