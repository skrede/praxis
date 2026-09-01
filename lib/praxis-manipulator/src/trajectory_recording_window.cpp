#include "praxis/manipulator/control_configuration.h"
#include "praxis/manipulator/trajectory_recording_window.h"

#include "praxis/extension/held_handle.h"

#include <imgui.h>

#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <vector>
#include <cstring>
#include <utility>
#include <filesystem>

namespace praxis::manipulator {

namespace {

constexpr const char *unpublished_arm = "The arm has published nothing yet.";

// Nothing answers the posted operation where it was posted, so the outcome is reported from inside
// it, on the strand that applied it. The folder is named as it resolved and not as it was typed, so
// a folder that is not the one meant is legible on sight.
void apply(robot_controller &control, const recording_parameters &requested)
{
    const expected<std::filesystem::path, refusal> where = control.set_recording(requested);
    if(!where.has_value())
        return;
    if(!requested.active && requested.directory.empty())
        return;

    if(where->empty())
        spdlog::info("praxis: the trajectory recording is {} and names no folder to write into", requested.active ? "on" : "off");
    else
        spdlog::info("praxis: the trajectory recording is {} and writes into '{}'", requested.active ? "on" : "off", where->string());
}

recording_parameters published_recording(const arm_reader &seen)
{
    return held(seen.read(), "the trajectory recording window", "published arm state").recording;
}

}

trajectory_recording_window::trajectory_recording_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm)
        : trajectory_recording_window(std::move(name), seen, std::move(arm), published_recording(seen))
{
}

trajectory_recording_window::trajectory_recording_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const settings &state, std::string at)
        : imgui_window(std::move(name))
        , m_active(state.active)
        , m_seen(std::move(seen))
        , m_directory_path{}
        , m_settings_at(std::move(at))
        , m_arm(std::move(arm))
{
    const std::string directory = state.directory.string();
    std::strncpy(m_directory_path, directory.c_str(), sizeof(m_directory_path) - 1u);
    command(m_arm, [state](robot_controller &control, scene_robot &) { apply(control, state); });
}

trajectory_recording_window::settings trajectory_recording_window::state() const
{
    settings parameters;
    parameters.active    = m_active;
    parameters.directory = m_directory_path;

    return parameters;
}

std::vector<config::edit> trajectory_recording_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_recording(state(), m_settings_at));
}

void trajectory_recording_window::render()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();

    ImGui::Begin(display_name().c_str());
    if(published == nullptr)
        ImGui::TextUnformatted(unpublished_arm);
    else
    {
        // The text field writes into the same buffer state() reads, so both controls hand the same
        // settings across the gate. Nothing can answer with the call, so the settings the recording
        // holds are the ones the next publication carries and the checkbox adopts them there.
        m_active           = published->recording.active;
        const bool toggled = ImGui::Checkbox("Active", &m_active);
        ImGui::InputText("Output folder", m_directory_path, sizeof(m_directory_path));
        const bool requested = ImGui::Button("Update folder");
        if(toggled || requested)
        {
            const settings asked = state();
            command(m_arm, [asked](robot_controller &control, scene_robot &) { apply(control, asked); });
        }
    }
    ImGui::End();
}

}
