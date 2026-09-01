#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/frame_roster_window.h"

#include <string>
#include <cstddef>

namespace praxis::rigid_motion::probe {

std::size_t unnamed_roster(frame_stencil &target)
{
    const frame_roster_window::selection_route standing{[] { return std::size_t{0}; }, [](std::size_t) {}};
    const frame_roster_window roster(std::string("Roster"), target, axes_settings{}, object_body{}, standing);

    return target.count();
}

}
