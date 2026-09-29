#include "frame_markers.h"
#include "arm_pose_transformations.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/pose_readout.h"
#include "praxis/manipulator/model_placement.h"
#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/slots.h"

#include "praxis/extension/coverage.h"

#include "praxis/scene/imgui_window.h"

#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace praxis::presets {

namespace {

using composed_windows = std::vector<std::shared_ptr<scene::imgui_window>>;

// Said once, at composition, rather than branched on at every read: the four transformations cannot
// refuse, so a window opened over an unbound one would show a plausible pose forever and no reader
// downstream could tell. Only the slots this scenario itself reads are counted, so binding a solver
// is not the price of attaching a tool. The models were chosen before this was reached, and a
// composition offering no window to hide them with draws neither.
composed_windows declined(manipulator::loadable_robot_stencil &on, const std::string &unbound)
{
    on.clear_flange_attachment(manipulator::flange_attachment::tool);
    on.clear_world_object();

    spdlog::error("praxis: 'presets.arm_windows_tooling' was denied {}, which still hold their defaults; every pose it would show would be answered without being computed, so it "
                  "composes no window and draws neither model",
                  unbound);

    return composed_windows{};
}

// The chain is the one this composition's own description derived, so a refusal here says it names a
// joint count the rendered arm does not have; the stencil names both counts where it compares them.
// The arm is still worth showing, so the windows are composed either way.
void draw_derived_chain(manipulator::loadable_robot_stencil &on, const manipulator::screw_chain &derived)
{
    const expected<void, refusal> told = on.set_joint_screws(derived.home, derived.space_screws);
    if(!told)
        spdlog::error("praxis: 'presets.arm_windows_tooling' was refused the chain its description derived; the arm is shown without its screw axes");
}

manipulator::robot_view_window::controls every_view_control()
{
    manipulator::robot_view_window::controls offered;
    offered.reach = true;

    offered.tool = true;

    return every_marker_control(offered);
}

void report_doubts(const char *model, const manipulator::placement_doubts &doubts)
{
    constexpr rigid_motion::frame_ops slot_names{};

    if(!doubts.defaulted.empty())
        spdlog::error("praxis: 'presets.arm_windows_tooling' placed {} through {}, which still hold their defaults, so it stands where their answers put it", model,
                      joined_slot_names(slot_names, doubts.defaulted));
    if(!doubts.not_finite.empty())
        spdlog::error("praxis: 'presets.arm_windows_tooling' placed {} through {}, which answered values that are not finite, so it stands where those values put it", model,
                      joined_slot_names(slot_names, doubts.not_finite));
}

// Both models are placed once the windows exist, so a window capturing a model on construction finds
// an inactive one still in the stencil.
composed_windows tooling_windows(const arm_scenario &state, const manipulator::arm_window_inputs &built)
{
    if(const std::string unbound = unbound_pose_transformations(built.inert); !unbound.empty())
        return declined(built.stencil, unbound);

    draw_derived_chain(built.stencil, built.chain);
    install_frame_markers(built.stencil);

    composed_windows windows{
            std::make_shared<manipulator::joint_control_window>("Joint control", built.seen, built.arm, state.joint_control, window_paths::joint_control),
            manipulator::compose_pose_readout("Pose", built.seen, built.frames, built.inert),
            std::make_shared<manipulator::tool_window>("Tool", built.stencil, built.seen, built.arm, built.frames, state.tool, window_paths::tool, state.model_roots),
            std::make_shared<manipulator::world_object_window>("World object", built.stencil, built.frames, state.world_object, window_paths::world_object, state.model_roots),
            std::make_shared<manipulator::robot_view_window>("View", built.stencil, every_view_control(), state.robot_view, window_paths::robot_view),
    };
    report_doubts("the tool", manipulator::seat_tool(built.stencil, built.arm, built.frames, state.tool));
    report_doubts("the world object", manipulator::place_world_object(built.stencil, built.frames, state.world_object));

    return windows;
}

}

manipulator::arm_composition arm_windows_tooling(arm_scenario chosen)
{
    manipulator::arm_composition composed;
    composed.draws_tool    = true;
    composed.draws_world   = true;
    composed.flange_marker = manipulator::flange_marker_policy::stands;
    composed.windows       = [state = std::move(chosen)](const manipulator::arm_window_inputs &built) { return tooling_windows(state, built); };

    return composed;
}

}
