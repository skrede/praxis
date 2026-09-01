#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_CAPABILITIES_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_CAPABILITIES_H

#include "praxis/rigid_motion/slots.h"

#include "praxis/extension/descriptor.h"

#include <array>

namespace praxis::rigid_motion {

struct capabilities
{
    frame_ops frame{};
    screw_ops screw{};
};

capabilities baseline();

// The views are in the aggregate's member order and point into the value passed, which must outlive
// them. A temporary argument would leave every view dangling at the end of the full expression, so
// that call is deleted rather than diagnosed at run time.
std::array<capability_view, 2> capability_views(const capabilities &c);
std::array<capability_view, 2> capability_views(capabilities &&) = delete;

}

#endif
