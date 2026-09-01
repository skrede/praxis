#ifndef HPP_GUARD_PRAXIS_PRESETS_ARRANGEMENT_SCENARIOS_H
#define HPP_GUARD_PRAXIS_PRESETS_ARRANGEMENT_SCENARIOS_H

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"

#include "praxis/config/document.h"

#include <memory>
#include <cstdint>
#include <functional>

namespace praxis::presets {

// euler_pose draws one controllable frame beside the fixed one; euler_relative draws two, each
// taking the fixed frame or the other as its parent; frame_workbench adds and removes frames while
// it runs; screw drives a pose along a screw; twist_axis names a screw axis by a twist; two_pose
// joins two poses by a screw; rotation_axis turns one frame about an axis through it. The order is
// the label table's, which is what reading a spelling back as an index and casting relies on.
enum class arrangement_scenario : std::uint8_t
{
    euler_pose,
    euler_relative,
    frame_workbench,
    screw,
    twist_axis,
    two_pose,
    rotation_axis
};

// The arrangement's own document is read where the arrangement is composed and handed to the
// composer, which owns what that document means for the scenario it builds.
using arrangement_composer = std::function<std::shared_ptr<scene::preset>(const scene::preset_site &, const config::document &)>;

// The capabilities are copied into the composer, so the argument need not outlive the call.
arrangement_composer composer_for(arrangement_scenario scenario, const rigid_motion::capabilities &motions);

arrangement_composer composer_for(arrangement_scenario scenario);

}

#endif
