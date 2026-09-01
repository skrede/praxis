#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_CAPABILITIES_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_CAPABILITIES_H

#include "praxis/trajectory/slots.h"

#include "praxis/extension/descriptor.h"

#include <array>

namespace praxis::trajectory {

struct capabilities
{
    time_scaling_ops time_scaling{};
    path_ops path{};
    pose_trajectory_ops pose_trajectory{};
    trajectory_ops trajectory{};
};

capabilities baseline();

// The views are in the aggregate's member order and point into the value passed, which must outlive
// them. A temporary argument would leave every view dangling at the end of the full expression, so
// that call is deleted rather than diagnosed at run time.
std::array<capability_view, 4> capability_views(const capabilities &c);
std::array<capability_view, 4> capability_views(capabilities &&) = delete;

}

#endif
