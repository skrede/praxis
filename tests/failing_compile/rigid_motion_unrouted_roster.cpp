#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/frame_roster_window.h"

#include <string>
#include <cstddef>

namespace praxis::rigid_motion::probe {

std::size_t unrouted_roster(frame_stencil &target)
{
    const frame_roster_window roster(std::string("Roster"), target, axes_settings{}, object_body{}, std::string("Frame"));

    return target.count();
}

}
