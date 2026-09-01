#ifndef HPP_GUARD_PRAXIS_PRESETS_EULER_RUNGS_H
#define HPP_GUARD_PRAXIS_PRESETS_EULER_RUNGS_H

#include "praxis/presets/arrangements.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"

#include <memory>
#include <cstdint>

namespace praxis::presets {

// single_frame draws one controllable frame beside the fixed one and offers no choice of what its
// placement is expressed in; paired_frames draws two and lets each take the fixed frame or the other
// as its parent. Either rung stands on one frame at a time: its parameters and both matrix readings
// are about the frame the selection names.
enum class euler_rung : std::uint8_t
{
    single_frame,
    paired_frames
};

std::shared_ptr<scene::preset> euler_rung_preset(const scene::preset_site &site, const rigid_motion::capabilities &motions, euler_rung rung,
                                                 arrangement_source arrangement = arrangement_source());

}

#endif
