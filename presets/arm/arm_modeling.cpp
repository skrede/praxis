#include "praxis/presets/arm.h"
#include "praxis/presets/screw_table.h"

#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/screw_modeling_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/imgui_window.h"

#include <spdlog/spdlog.h>

#include <memory>
#include <vector>
#include <utility>

namespace praxis::presets {

namespace {

// The rendered arm is the independent reference a wrong chain is read against, so the control that
// hides it is offered and can always be taken back; the drawn axes carry no control at all, since a
// scenario whose subject is the chain has nothing left to show once they are gone.
manipulator::robot_view_window::controls chain_view_controls()
{
    manipulator::robot_view_window::controls offered;
    offered.reach      = true;
    offered.decoration = false;

    return offered;
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

}

manipulator::arm_composition arm_windows_modeling(arm_scenario chosen, screw_table_source keeping)
{
    manipulator::arm_composition composed;
    composed.windows = [state = std::move(chosen), kept = std::move(keeping)](const manipulator::arm_window_inputs &built)
    {
        built.stencil.set_flange_attachment(manipulator::flange_attachment::frame_marker, manipulator::make_flange_marker(built.stencil.robot()));

        // The document this scenario announces declares the chain's keys and no others, so a window
        // beside the chain names no key path: an edit written against a declaration that does not
        // name it makes the whole save write nothing at all.
        return std::vector<std::shared_ptr<scene::imgui_window>>{
                std::make_shared<manipulator::joint_control_window>("Joint control", built.seen, built.arm, state.joint_control),
                std::make_shared<manipulator::screw_modeling_window>("Chain", built.stencil, built.seen, built.screw, built.frames, built.fk, built.chain,
                                                                     manipulator::screw_modeling_window::controls(), opened_chain(kept, built), screw_table_edits(built.frames),
                                                                     kept.save, kept.at),
                std::make_shared<manipulator::robot_view_window>("View", built.stencil, chain_view_controls(), chain_view(state.robot_view)),
        };
    };

    return composed;
}

}
