#ifndef HPP_GUARD_PRAXIS_PRESETS_SCREW_H
#define HPP_GUARD_PRAXIS_PRESETS_SCREW_H

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"

#include <memory>

namespace praxis::presets {

std::shared_ptr<scene::preset> screw_preset(const scene::preset_site &site, const rigid_motion::capabilities &motions);

}

#endif
