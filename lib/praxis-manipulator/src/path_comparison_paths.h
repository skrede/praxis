#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_PATH_COMPARISON_PATHS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_PATH_COMPARISON_PATHS_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/path_comparison_window.h"

#include "praxis/trajectory/path.h"

#include <vector>

namespace praxis::manipulator {

// The configurations the joint-space shape passes through, at the shared count and the shared path
// parameters. Empty where the shape refused, so a caller runs what it was given rather than a run
// with holes in it.
std::vector<joint_vector> configurations_along(const trajectory::path_ops &shapes, const joint_vector &from, const joint_vector &to);

}

#endif
