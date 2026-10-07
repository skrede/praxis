#include "frame_markers.h"

#include "praxis/presets/arm.h"
#include "praxis/presets/screw_table.h"
#include "praxis/presets/kept_modeling.h"

#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/pose_readout.h"
#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/tool_configuration.h"
#include "praxis/manipulator/view_configuration.h"
#include "praxis/manipulator/screw_modeling_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <spdlog/spdlog.h>

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

namespace praxis::presets {

namespace {

using composed_windows = std::vector<std::shared_ptr<scene::imgui_window>>;

manipulator::robot_view_window::controls chain_view_controls(modeling_beside beside)
{
    manipulator::robot_view_window::controls offered;
    offered.reach                  = true;
    offered.decoration             = true;
    offered.tool                   = beside == modeling_beside::pose_and_tool;
    offered.described_frame_marker = true;

    return every_marker_control(offered);
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

// The View and Tool windows write under the paths every arm document declares them at, and the screw
// table's declaration names both, so a document keeping the chain keeps them beside it.
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
        opened.push_back(std::make_shared<manipulator::tool_window>("Tool", built.stencil, built.seen, built.arm, built.frames, state.tool, window_paths::tool, state.model_roots));
    opened.push_back(std::make_shared<manipulator::robot_view_window>("View", built.stencil, chain_view_controls(beside), state.robot_view, window_paths::robot_view));

    return opened;
}

// A document carrying none of these keys reads what the caller opened at.
config::declaration falling_back_to(const config::declaration &declared, std::span<const config::edit> opened)
{
    config::declaration shape(declared.space());
    for(const config::node &held : declared.nodes())
    {
        const auto named     = std::ranges::find(opened, held.path, &config::edit::key);
        std::string fallback = named == opened.end() ? held.fallback : named->value;
        if(held.shape == config::node_kind::group)
            shape.group(held.path);
        else if(held.shape == config::node_kind::collection)
            shape.collection(held.path, held.identity);
        else if(!held.allowed.empty())
            shape.choice(held.path, held.allowed, std::move(fallback));
        else
            shape.field(held.path, held.kind, std::move(fallback));
    }

    return shape;
}

std::vector<config::edit> opened_beside_the_chain(const arm_scenario &machine)
{
    std::vector<config::edit> opened       = manipulator::write_robot_view(machine.robot_view, window_paths::robot_view);
    const std::vector<config::edit> tooled = manipulator::write_tool(machine.tool, window_paths::tool);
    opened.insert(opened.end(), tooled.begin(), tooled.end());

    return opened;
}

arm_scenario reopened_beside_the_chain(const arm_scenario &machine, const config::document &kept)
{
    arm_scenario opening = machine;
    opening.robot_view   = manipulator::read_robot_view(kept, window_paths::robot_view);
    opening.tool         = manipulator::read_tool(kept, window_paths::tool);

    return opening;
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

kept_modeling arm_windows_kept_modeling(const arm_scenario &chosen, const config::binding &keeping, modeling_beside beside)
{
    const config::binding bound{falling_back_to(keeping.shape, opened_beside_the_chain(chosen)), keeping.at, keeping.carries};
    const config::outcome kept = config::load_or_defaults(bound);
    const arm_scenario opening = reopened_beside_the_chain(chosen, kept.values);
    const screw_table_source supplied{screw_table_path, kept.values, bound};

    return kept_modeling{opening, bound, kept.values, arm_windows_modeling(opening, supplied, beside)};
}

}
