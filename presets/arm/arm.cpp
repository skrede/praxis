#include "frame_markers.h"
#include "arm_pose_transformations.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/edited_pose.h"
#include "praxis/manipulator/pose_readout.h"
#include "praxis/manipulator/world_object_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"
#include "praxis/manipulator/trajectory_recording_window.h"

#include "praxis/scene/imgui_window.h"

#include <meios/urdf/load.h>

#include <spdlog/spdlog.h>

#include <threepp/threepp.hpp>

#include <format>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <string_view>

namespace praxis::presets {

namespace {

using composed_windows = std::vector<std::shared_ptr<scene::imgui_window>>;

constexpr std::string_view composer_name = "presets.arm_windows";

composed_windows declined(manipulator::loadable_robot_stencil &on, const std::string &unbound)
{
    on.clear_flange_attachment(manipulator::flange_attachment::tool);
    on.clear_world_object();

    spdlog::error("praxis: '{}' was denied {}, which still hold their defaults; every pose it would show or command would be answered without being computed, so it composes no "
                  "window and draws neither model",
                  composer_name, unbound);

    return composed_windows{};
}

std::shared_ptr<threepp::Object3D> loaded_mesh(const std::string &path)
{
    if(path.empty())
        return nullptr;

    threepp::STLLoader loader;
    const auto geometry = loader.load(path);
    if(geometry == nullptr)
    {
        spdlog::error(std::format("Loading model {} failed", path));

        return nullptr;
    }

    return threepp::Mesh::create(geometry, threepp::MeshPhongMaterial::create({{"flatShading", true}, {"color", threepp::Color::gray}}));
}

// A node belongs to one scene at a time, so a composition is given meshes of its own rather than
// meshes another composition of the same description is already showing. A model the composition
// declares it does not draw is not loaded at all, so nothing stands in a scene with no control over
// it: what is drawn is decided where the windows are decided and nowhere here.
manipulator::attachments scenario_attachments(const manipulator::arm_composition &composed, const arm_scenario &chosen)
{
    return manipulator::attachments{composed.draws_tool ? loaded_mesh(chosen.tool.model_path) : nullptr, composed.draws_world ? loaded_mesh(chosen.world_object.model_path) : nullptr,
                                    composed.flange_marker};
}

}

manipulator::arm_composition arm_windows(arm_scenario chosen)
{
    manipulator::arm_composition composed;
    composed.draws_tool  = true;
    composed.draws_world = true;
    composed.windows     = [state = std::move(chosen)](const manipulator::arm_window_inputs &built)
    {
        if(const std::string unbound = unbound_pose_transformations(built.inert); !unbound.empty())
            return declined(built.stencil, unbound);

        install_frame_markers(built.stencil);

        // The task-space window and the two jog windows drive one pose between them, so the three are
        // handed the same one.
        auto edited = std::make_shared<manipulator::edited_pose>();

        return composed_windows{
                std::make_shared<manipulator::world_object_window>("World object settings", built.stencil, built.frames, state.world_object, window_paths::world_object),
                std::make_shared<manipulator::joint_control_window>("Joint control", built.seen, built.arm, state.joint_control, window_paths::joint_control),
                std::make_shared<manipulator::task_space_window>("Task space", built.seen, built.arm, built.frames, edited, state.task_space, window_paths::task_space),
                std::make_shared<manipulator::tool_jog_window>("Tool frame jog", built.seen, built.arm, built.frames, edited, state.tool_jog, window_paths::tool_jog),
                std::make_shared<manipulator::screw_jog_window>("Screw jog", built.seen, built.arm, built.frames, edited, state.screw_jog, window_paths::screw_jog),
                std::make_shared<manipulator::control_parameters_window>("Control parameters", built.seen, built.arm, state.parameters, window_paths::parameters),
                manipulator::compose_pose_readout("Pose##1", built.seen, built.frames, built.inert),
                manipulator::compose_pose_readout("Pose##2", built.seen, built.frames, built.inert),
                std::make_shared<manipulator::tool_window>("Tool settings", built.stencil, built.seen, built.arm, built.frames, state.tool, window_paths::tool),
                std::make_shared<manipulator::trajectory_recording_window>("Recording", built.seen, built.arm, state.recording, window_paths::recording),
        };
    };

    return composed;
}

std::shared_ptr<scene::preset> arm_preset(const scene::preset_site &site, const manipulator::capabilities &arm, const trajectory::capabilities &shapes,
                                          const rigid_motion::capabilities &motions, const arm_scenario &chosen, const manipulator::arm_composition &composed)
{
    auto description = meios::load(chosen.description, chosen.options);
    if(!description)
    {
        const auto &failure = description.error();
        spdlog::error(std::format("Loading description {} failed: {} with code {}", chosen.description.string(), failure.message, static_cast<int>(failure.code)));

        return nullptr;
    }

    return manipulator::compose_arm(description->robot, site, scenario_attachments(composed, chosen), arm, shapes, motions, chosen.initial, composed.windows);
}

}
