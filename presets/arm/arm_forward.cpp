#include "praxis/presets/arm.h"

#include "praxis/manipulator/pose_readout.h"
#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/imgui_window.h"

#include <spdlog/spdlog.h>

#include <memory>
#include <vector>
#include <utility>

namespace praxis::presets {

namespace {

// The chain is the one this composition's own description derived, so a refusal here says it names a
// joint count the rendered arm does not have; the stencil names both counts where it compares them.
// The arm is still worth showing, so the windows are composed either way.
void draw_derived_chain(manipulator::loadable_robot_stencil &on, const manipulator::screw_chain &derived)
{
    const expected<void, refusal> told = on.set_joint_screws(derived.home, derived.space_screws);
    if(!told)
        spdlog::error("praxis: 'presets.arm_windows_forward' was refused the chain its description derived; the arm is shown without its screw axes");
}

manipulator::robot_view_window::controls every_view_control()
{
    manipulator::robot_view_window::controls offered;
    offered.reach = true;

    return offered;
}

}

manipulator::arm_composition arm_windows_forward(arm_scenario chosen)
{
    manipulator::arm_composition composed;
    composed.windows = [state = std::move(chosen)](const manipulator::arm_window_inputs &built)
    {
        draw_derived_chain(built.stencil, built.chain);
        built.stencil.set_flange_attachment(manipulator::flange_attachment::frame_marker, manipulator::make_flange_marker(built.stencil.robot()));

        return std::vector<std::shared_ptr<scene::imgui_window>>{
                std::make_shared<manipulator::joint_control_window>("Joint control", built.seen, built.arm, state.joint_control, window_paths::joint_control),
                manipulator::compose_pose_readout("Pose", built.seen, built.frames, built.inert),
                std::make_shared<manipulator::robot_view_window>("View", built.stencil, every_view_control(), state.robot_view, window_paths::robot_view),
        };
    };

    return composed;
}

}
