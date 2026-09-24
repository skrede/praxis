#ifndef HPP_GUARD_PRAXIS_PRESETS_ARM_FRAME_MARKERS_H
#define HPP_GUARD_PRAXIS_PRESETS_ARM_FRAME_MARKERS_H

#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

namespace praxis::presets {

// Both frame markers a view window carries a switch for, so a scenario offering either switch has a
// marker for it to reach. The one at the tool frame is carried at the tool offset the arm publishes,
// so it coincides with the one at the flange while that offset is the identity.
inline void install_frame_markers(manipulator::loadable_robot_stencil &on)
{
    on.set_flange_attachment(manipulator::flange_attachment::frame_marker, manipulator::make_flange_marker(on.robot()));
    on.set_flange_attachment(manipulator::flange_attachment::tool_frame_marker, manipulator::make_flange_marker(on.robot()));
}

// Every switch and every size the view window offers over what the arm draws at a frame.
inline manipulator::robot_view_window::controls every_marker_control(manipulator::robot_view_window::controls offered)
{
    offered.flange_marker     = true;
    offered.tool_frame_marker = true;
    offered.marker_scale      = true;

    return offered;
}

}

#endif
