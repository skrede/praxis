#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/tool_jog_window.h"
#include "praxis/manipulator/control_configuration.h"

#include "praxis/extension/held_handle.h"

#include "praxis/scene/widgets.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/axis_order.h"

#include <imgui.h>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace praxis::manipulator {

namespace {

constexpr std::array<control_mode, 2> mode_options{control_mode::preview, control_mode::simulation};

const char *const position_labels[3]{"X", "Y", "Z"};
const char *const orientation_labels[3]{"A", "B", "C"};

constexpr const char *unpublished_arm = "The arm has published nothing yet.";

// The jog's own rotation is composed about the tool frame's axes in this order; the order selector on
// screen names the order of the start pose the panel shares, not this one.
constexpr axis_order jog_order = axis_order::zyx;

}

tool_jog_window::settings::settings(control_mode chosen)
        : mode(chosen)
{
}

tool_jog_window::tool_jog_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited)
        : tool_jog_window(std::move(name), std::move(seen), std::move(arm), injected, std::move(edited), settings{})
{
}

tool_jog_window::tool_jog_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited,
                                 const settings &state, std::string at)
        : imgui_window(std::move(name))
        , m_seen(seen)
        , m_settings_at(std::move(at))
        , m_arm(std::move(arm))
        , m_jog_position(Eigen::Vector3f::Zero())
        , m_frame(injected)
        , m_jog_euler_degrees(Eigen::Vector3f::Zero())
        , m_edited(std::move(edited))
        , m_control_mode(state.mode, mode_options, control_mode_labels())
{
    static_cast<void>(held(seen.read(), "the tool jog window", "published arm state"));
    static_cast<void>(held(m_edited, "the tool jog window", "shared pose"));
}

tool_jog_window::settings tool_jog_window::state() const
{
    return settings{m_control_mode.value()};
}

std::vector<config::edit> tool_jog_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_tool_jog(state(), m_settings_at));
}

void tool_jog_window::render()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();

    ImGui::Begin(display_name().c_str());
    if(published == nullptr)
        ImGui::TextUnformatted(unpublished_arm);
    else
    {
        render_option_cycle("Control mode", m_control_mode);
        render_tool_frame_jog(*published);
    }
    ImGui::End();
}

bool tool_jog_window::render_jog_start_pose(const arm_snapshot &seen)
{
    bool moved = ImGui::InputFloat3("XYZ", m_edited->position.data());
    moved      = ImGui::InputFloat3("ABC", m_edited->euler_degrees.data()) || moved;
    moved      = scene::render_enum_selection("Euler order", m_edited->order, axis_order_labels()) || moved;
    if(!ImGui::Button("Reset") || !seed_from(*m_edited, seen, m_frame))
        return moved;

    m_jog_position      = Eigen::Vector3f::Zero();
    m_jog_euler_degrees = Eigen::Vector3f::Zero();

    return moved;
}

void tool_jog_window::render_tool_frame_jog(const arm_snapshot &seen)
{
    const auto jog = [this](int)
    {
        if(m_control_mode != control_mode::preview)
            return;

        const Eigen::Vector3d angles = m_jog_euler_degrees.cast<double>() * radians_per_degree;
        const transform start        = pose_matrix(*m_edited, m_frame);
        const Eigen::Vector3d offset = m_jog_position.cast<double>();
        const rotation turned        = m_frame.rotation_matrix_from_euler(angles, jog_order);
        command(m_arm, [start, offset, turned](robot_controller &control, scene_robot &) { control.preview_tool_frame_jog(start, offset, turned); });
    };
    if(render_jog_start_pose(seen))
        jog(0);
    scene::render_float3_slider(m_jog_position, position_labels, -1.f, 1.f, jog);
    scene::render_float3_slider(m_jog_euler_degrees, orientation_labels, -180.f, 180.f, jog);
}

}
