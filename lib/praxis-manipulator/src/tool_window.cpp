#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/tool_configuration.h"

#include "praxis/scene/widgets.h"

#include <array>
#include <string>
#include <memory>
#include <vector>
#include <cstring>
#include <utility>

namespace praxis::manipulator {

namespace {

constexpr const char *absent_tool_line = "No tool model is attached.";

constexpr std::array<tool_window::tool_view, 3> tool_views{tool_window::tool_view::kinematics_transform, tool_window::tool_view::graphics_transform, tool_window::tool_view::load_stl};
constexpr std::array<const char *, 3> tool_view_labels{"Kinematics transform", "Graphics transform", "Load .stl"};

}

tool_window::settings::settings(bool chosen_active, std::string chosen_model_path, tool_view chosen_view, const Eigen::Vector3f &chosen_gfx_euler_degrees,
                                axis_order chosen_gfx_euler_order, const Eigen::Vector3f &chosen_gfx_scale, const Eigen::Vector3f &chosen_gfx_offset,
                                const Eigen::Vector3f &chosen_kinematics_euler_degrees, axis_order chosen_kinematics_euler_order, const Eigen::Vector3f &chosen_kinematics_offset)
        : active(chosen_active)
        , model_path(std::move(chosen_model_path))
        , selected_view(chosen_view)
        , gfx_euler_degrees(chosen_gfx_euler_degrees)
        , gfx_euler_order(chosen_gfx_euler_order)
        , gfx_scale(chosen_gfx_scale)
        , gfx_offset(chosen_gfx_offset)
        , kinematics_euler_degrees(chosen_kinematics_euler_degrees)
        , kinematics_euler_order(chosen_kinematics_euler_order)
        , kinematics_offset(chosen_kinematics_offset)
{
}

tool_window::tool_window(std::string name, loadable_robot_stencil &stencil, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected)
        : tool_window(std::move(name), stencil, std::move(seen), std::move(arm), injected, settings{})
{
}

tool_window::tool_window(std::string name, loadable_robot_stencil &stencil, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected,
                         const settings &state, std::string at)
        : imgui_window(std::move(name))
        , m_active(state.active)
        , m_seen(std::move(seen))
        , m_model_path{}
        , m_settings_at(std::move(at))
        , m_gfx_euler_degrees(state.gfx_euler_degrees)
        , m_gfx_scale(state.gfx_scale)
        , m_gfx_offset(state.gfx_offset)
        , m_tool_euler_degrees(state.kinematics_euler_degrees)
        , m_gfx_euler_order(state.gfx_euler_order)
        , m_tool_offset(state.kinematics_offset)
        , m_tool_euler_order(state.kinematics_euler_order)
        , m_arm(std::move(arm))
        , m_tool_view(state.selected_view, tool_views, tool_view_labels)
        , m_chosen_view(state.selected_view)
        , m_stencil(stencil)
        , m_tool(stencil.attached_at(flange_attachment::tool))
        , m_frame(injected)
{
    std::strncpy(m_model_path, state.model_path.c_str(), sizeof(m_model_path) - 1u);
}

tool_window::settings tool_window::state() const
{
    return settings{
            m_active, m_model_path, m_tool_view.value(), m_gfx_euler_degrees, m_gfx_euler_order, m_gfx_scale, m_gfx_offset, m_tool_euler_degrees, m_tool_euler_order, m_tool_offset,
    };
}

std::vector<config::edit> tool_window::settings_edits(const config::document &carried) const
{
    settings chosen      = state();
    chosen.selected_view = m_chosen_view;

    return config::unsaved_edits(carried, write_tool(chosen, m_settings_at));
}

void tool_window::render()
{
    ImGui::Begin(display_name().c_str());
    if(m_tool != nullptr)
        render_activation();

    if(render_option_cycle("Tool view", m_tool_view))
        m_chosen_view = m_tool_view.value();
    if(m_tool_view == tool_view::graphics_transform)
        render_graphics_transform();
    else if(m_tool_view == tool_view::kinematics_transform)
        render_kinematics_transform();
    else
        render_stl_loader();
    ImGui::End();
}

// The starting state is the tool the stencil was built with. Both transform panes drive that tool,
// so a window holding none opens at the loader whatever view it was given.
void tool_window::initialize()
{
    if(m_active && m_tool != nullptr)
    {
        seat_attached_tool();

        return;
    }

    activate_default_tool();
    if(m_tool == nullptr)
        m_tool_view.set(tool_view::load_stl);
}

void tool_window::seat_attached_tool()
{
    assign_gfx_transform();
    assign_kinematics_transform();
}

void tool_window::render_activation()
{
    if(ImGui::Checkbox("Active", &m_active))
    {
        if(m_active)
            activate_custom_tool();
        else
            activate_default_tool();
    }
    ImGui::Text("Loaded model: %s", m_model_path);
}

void tool_window::render_stl_loader()
{
    ImGui::InputText("STL file", m_model_path, sizeof(m_model_path));
    if(ImGui::Button("Load"))
    {
        m_active = load_stl();
        if(m_active)
        {
            m_tool_view.set(tool_view::kinematics_transform);
            m_chosen_view = m_tool_view.value();
            activate_custom_tool();
        }
    }
}

void tool_window::render_kinematics_transform()
{
    const char *const position_labels[3]{"X", "Y", "Z"};
    scene::render_float3_inputs(m_tool_offset, position_labels, 0.01f, 0.1f);
    ImGui::NewLine();
    render_euler_inputs("TCP euler order", m_tool_euler_degrees, m_tool_euler_order, 1.f, 10.f);
    if(ImGui::Button("Set"))
        assign_kinematics_transform();
}

void tool_window::render_graphics_transform()
{
    if(m_tool == nullptr)
    {
        ImGui::TextUnformatted(absent_tool_line);

        return;
    }

    const char *const scale_labels[3]{"SX", "SY", "SZ"};
    const char *const position_labels[3]{"X", "Y", "Z"};
    scene::render_float3_inputs(m_gfx_offset, position_labels, 0.01f, 0.1f);
    ImGui::NewLine();
    render_euler_inputs("GFX euler order", m_gfx_euler_degrees, m_gfx_euler_order, 1.f, 10.f);
    ImGui::NewLine();
    scene::render_float3_inputs(m_gfx_scale, scale_labels, 0.01f, 0.1f);
    if(ImGui::Button("Set"))
        assign_gfx_transform();
}

}
