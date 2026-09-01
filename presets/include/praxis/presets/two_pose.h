#ifndef HPP_GUARD_PRAXIS_PRESETS_TWO_POSE_H
#define HPP_GUARD_PRAXIS_PRESETS_TWO_POSE_H

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"

#include <memory>

namespace praxis::presets {

// A start pose and an end pose, the screw that joins them, and a body carried along it by a
// parameter control. Beside it stands the path the same two poses give when the orientation and the
// position are carried independently, so the two can be read against each other. Both are asked of
// the bound operations.
std::shared_ptr<scene::preset> two_pose_preset(const scene::preset_site &site, const rigid_motion::capabilities &motions);

}

#endif
