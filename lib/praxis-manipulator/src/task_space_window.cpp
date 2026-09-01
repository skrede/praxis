#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/task_space_window.h"
#include "praxis/manipulator/control_configuration.h"

#include "praxis/extension/held_handle.h"

#include "praxis/scene/widgets.h"

#include "praxis/rigid_motion/axis_order.h"

#include <imgui.h>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace praxis::manipulator {

namespace {

using shape = task_space_window::motion_shape;

constexpr std::array<shape, 2> shape_options{shape::ptp, shape::lin};
constexpr std::array<const char *, 2> shape_labels{"P2P", "LIN"};

constexpr std::array<control_mode, 2> mode_options{control_mode::preview, control_mode::simulation};

const char *const position_labels[3]{"X", "Y", "Z"};
const char *const orientation_labels[3]{"A", "B", "C"};

constexpr const char *unpublished_arm = "The arm has published nothing yet.";

}

task_space_window::settings::settings(motion_shape chosen_shape, control_mode chosen_mode)
        : shape(chosen_shape)
        , mode(chosen_mode)
{
}

task_space_window::task_space_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited)
        : task_space_window(std::move(name), std::move(seen), std::move(arm), injected, std::move(edited), settings{})
{
}

task_space_window::task_space_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited,
                                     const settings &state, std::string at)
        : imgui_window(std::move(name))
        , m_seen(seen)
        , m_settings_at(std::move(at))
        , m_arm(std::move(arm))
        , m_frame(injected)
        , m_edited(std::move(edited))
        , m_control_mode(state.mode, mode_options, control_mode_labels())
        , m_motion_shape(state.shape, shape_options, shape_labels)
{
    static_cast<void>(held(seen.read(), "the task space window", "published arm state"));
    static_cast<void>(held(m_edited, "the task space window", "shared pose"));
}

task_space_window::settings task_space_window::state() const
{
    return settings{m_motion_shape.value(), m_control_mode.value()};
}

std::vector<config::edit> task_space_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_task_space(state(), m_settings_at));
}

void task_space_window::render()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();

    ImGui::Begin(display_name().c_str());
    if(published == nullptr)
        ImGui::TextUnformatted(unpublished_arm);
    else
        render_task_space(*published);
    ImGui::End();
}

void task_space_window::render_task_space(const arm_snapshot &seen)
{
    render_option_cycle("Control mode", m_control_mode);
    render_option_cycle("Trajectory", m_motion_shape);

    if(m_control_mode == control_mode::preview)
        render_task_space_preview(seen);
    else
        render_task_space_lin_p2p(seen);
}

void task_space_window::render_task_space_preview(const arm_snapshot &seen)
{
    const auto preview = [this](int)
    {
        const transform target = pose_matrix(*m_edited, m_frame);
        command(m_arm, [target](robot_controller &control, scene_robot &) { control.preview_task_space_pose(target); });
    };
    scene::render_float3_slider(m_edited->position, position_labels, -1.f, 1.f, preview);
    ImGui::NewLine();
    scene::render_float3_slider_with_reset(m_edited->euler_degrees, orientation_labels, -180.f, 180.f, preview);
    if(scene::render_enum_selection("Euler order", m_edited->order, axis_order_labels()))
        preview(0);
    if(ImGui::Button("Reset to current") && seed_from(*m_edited, seen, m_frame))
        preview(0);
}

void task_space_window::render_task_space_lin_p2p(const arm_snapshot &seen)
{
    scene::render_float3_inputs(m_edited->position, position_labels, 0.01f, 0.1f);
    ImGui::NewLine();
    render_euler_inputs("Euler order", m_edited->euler_degrees, m_edited->order, 0.01f, 0.1f);
    if(ImGui::Button("Reset to current"))
        seed_from(*m_edited, seen, m_frame);
    ImGui::SameLine();
    if(!ImGui::Button("Move"))
        return;

    const transform target = pose_matrix(*m_edited, m_frame);
    const bool straight    = m_motion_shape == motion_shape::lin;
    command(m_arm,
            [target, straight](robot_controller &control, scene_robot &)
            {
                if(straight)
                    control.task_space_lin(target);
                else
                    control.task_space_ptp(target);
            });
}

}
