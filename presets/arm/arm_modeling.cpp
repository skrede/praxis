#include "frame_markers.h"

#include "praxis/presets/arm.h"
#include "praxis/presets/screw_table.h"

#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/pose_readout.h"
#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/screw_modeling_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/imgui_window.h"

#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace praxis::presets {

namespace {

using composed_windows = std::vector<std::shared_ptr<scene::imgui_window>>;

// The rendered arm is the independent reference a wrong chain is read against, so the control that
// hides it is offered and can always be taken back; the drawn axes carry no control at all, since a
// scenario whose subject is the chain has nothing left to show once they are gone.
manipulator::robot_view_window::controls chain_view_controls(modeling_beside beside)
{
    manipulator::robot_view_window::controls offered;
    offered.reach                  = true;
    offered.decoration             = false;
    offered.tool                   = beside == modeling_beside::pose_and_tool;
    offered.described_frame_marker = true;

    return every_marker_control(offered);
}

manipulator::robot_view_window::settings chain_view(manipulator::robot_view_window::settings chosen)
{
    chosen.decoration = true;

    return chosen;
}

manipulator::screw_modeling_window::settings opened_chain(const screw_table_source &keeping, const manipulator::arm_window_inputs &built)
{
    if(!keeping.values)
        return manipulator::screw_modeling_window::settings{};

    const expected<manipulator::screw_modeling_window::settings, config::error> read = read_screw_table(*keeping.values, keeping.at, built.chain, built.screw, built.frames);
    if(read)
        return read.value();

    spdlog::error("praxis: the chain kept for this machine was not opened, so the scenario opens at none: {}", read.error().message);

    return manipulator::screw_modeling_window::settings{};
}

// The document this scenario announces declares the chain's keys and no others, so a window beside
// the chain names no key path: an edit written against a declaration that does not name it makes
// the whole save write nothing at all.
composed_windows beside_the_chain(const arm_scenario &state, const screw_table_source &kept, const manipulator::arm_window_inputs &built, modeling_beside beside)
{
    composed_windows opened{
            std::make_shared<manipulator::joint_control_window>("Joint control", built.seen, built.arm, state.joint_control),
            std::make_shared<manipulator::screw_modeling_window>("Chain", built.stencil, built.seen, built.screw, built.frames, built.fk, built.chain,
                                                                 manipulator::screw_modeling_window::controls(), opened_chain(kept, built), screw_table_edits(built.chain, built.frames),
                                                                 screw_table_route(kept.into, built.chain, built.frames), kept.at),
            manipulator::compose_pose_readout("Pose", built.seen, built.frames, built.inert, built.stencil),
    };
    if(beside == modeling_beside::pose_and_tool)
        opened.push_back(std::make_shared<manipulator::tool_window>("Tool", built.stencil, built.seen, built.arm, built.frames, state.tool, std::string(), state.model_roots));
    opened.push_back(std::make_shared<manipulator::robot_view_window>("View", built.stencil, chain_view_controls(beside), chain_view(state.robot_view)));

    return opened;
}

}

manipulator::arm_composition arm_windows_modeling(arm_scenario chosen, screw_table_source keeping, modeling_beside beside)
{
    manipulator::arm_composition composed;
    composed.draws_tool = beside == modeling_beside::pose_and_tool;
    composed.windows    = [state = std::move(chosen), kept = std::move(keeping), beside](const manipulator::arm_window_inputs &built)
    {
        install_frame_markers(built.stencil);
        install_described_marker(built.stencil);

        return beside_the_chain(state, kept, built, beside);
    };

    return composed;
}

}
