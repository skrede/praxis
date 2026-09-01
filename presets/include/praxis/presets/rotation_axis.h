#ifndef HPP_GUARD_PRAXIS_PRESETS_ROTATION_AXIS_H
#define HPP_GUARD_PRAXIS_PRESETS_ROTATION_AXIS_H

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"

#include <memory>

namespace praxis::presets {

// One frame standing at the origin and turned about an axis running through it, set by a direction
// and an angle control. Beside the frame stand two arrows out of the origin along that axis: one
// drawn at the length the frame's own arrows are drawn at, and one drawn at the angle times that
// same length. Each end of the frame's own three arrows draws the arc it has already travelled, from
// no turn at all to the turn now commanded, in that arrow's own tone; an arc grows and shrinks with
// the angle rather than spanning the whole turn the control can reach, so its extent is a second
// reading of the angle the coordinate arrow's length is the first of. The turn is asked of the bound
// operations. Each of the four drawings has a switch of its own.
std::shared_ptr<scene::preset> rotation_axis_preset(const scene::preset_site &site, const rigid_motion::capabilities &motions);

}

#endif
