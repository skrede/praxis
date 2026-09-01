#ifndef HPP_GUARD_PRAXIS_PRESETS_TWIST_AXIS_H
#define HPP_GUARD_PRAXIS_PRESETS_TWIST_AXIS_H

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"

#include <memory>

namespace praxis::presets {

// A twist set component by component, the screw axis it names drawn where that axis runs, and a
// body swept along it by an angle control. The axis is asked of the bound operations.
std::shared_ptr<scene::preset> twist_axis_preset(const scene::preset_site &site, const rigid_motion::capabilities &motions);

}

#endif
