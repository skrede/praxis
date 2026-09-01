#ifndef HPP_GUARD_PRAXIS_PRESETS_FRAME_WORKBENCH_H
#define HPP_GUARD_PRAXIS_PRESETS_FRAME_WORKBENCH_H

#include "praxis/presets/arrangements.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"

#include <memory>

namespace praxis::presets {

// A frame tree built while the application runs: a roster creates, names and removes frames, the
// placement panel follows the frame set as it grows and shrinks, and the matrix readouts read
// whichever frame the roster stands on. The frame the others are measured against is object zero and
// no panel moves it.
std::shared_ptr<scene::preset> frame_workbench_preset(const scene::preset_site &site, const rigid_motion::capabilities &motions, arrangement_source arrangement = arrangement_source());

}

#endif
