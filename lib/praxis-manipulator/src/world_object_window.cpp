#include "configuration_keys.h"

#include "praxis/manipulator/model_file.h"
#include "praxis/manipulator/model_placement.h"
#include "praxis/manipulator/tool_configuration.h"
#include "praxis/manipulator/world_object_window.h"

#include "praxis/scene/widgets.h"

#include <string>
#include <vector>
#include <cstring>
#include <utility>
#include <optional>
#include <filesystem>

namespace praxis::manipulator {

world_object_window::settings::settings(bool chosen_active, std::string chosen_model_path, world_view chosen_view, const Eigen::Vector3f &chosen_gfx_scale,
                                        const Eigen::Vector3f &chosen_gfx_offset, const Eigen::Vector3f &chosen_gfx_euler_zyx_degrees)
        : active(chosen_active)
        , model_path(std::move(chosen_model_path))
        , selected_view(chosen_view)
        , gfx_scale(chosen_gfx_scale)
        , gfx_offset(chosen_gfx_offset)
        , gfx_euler_zyx_degrees(chosen_gfx_euler_zyx_degrees)
{
}

world_object_window::world_object_window(std::string name, loadable_robot_stencil &target, const rigid_motion::frame_ops &injected)
        : world_object_window(std::move(name), target, injected, settings{})
{
}

world_object_window::world_object_window(std::string name, loadable_robot_stencil &target, const rigid_motion::frame_ops &injected, const settings &state, std::string at,
                                         std::vector<std::filesystem::path> roots)
        : imgui_window(std::move(name))
        , m_active(state.active)
        , m_model_path{}
        , m_settings_at(std::move(at))
        , m_gfx_scale(state.gfx_scale)
        , m_gfx_offset(state.gfx_offset)
        , m_gfx_euler_zyx_degrees(state.gfx_euler_zyx_degrees)
        , m_world_view(state.selected_view, {world_view::transform, world_view::load_stl}, {"Transform", "Load .stl"})
        , m_chosen_view(state.selected_view)
        , m_stencil(target)
        , m_frame(injected)
        , m_world_object(target.world_object())
        , m_roots(std::move(roots))
        , m_loaded(loaded_model_line(state.model_path, m_roots, m_world_object != nullptr))
{
    std::strncpy(m_model_path, state.model_path.c_str(), sizeof(m_model_path) - 1u);
}

world_object_window::settings world_object_window::state() const
{
    return settings{
            m_active, m_model_path, m_world_view.value(), m_gfx_scale, m_gfx_offset, m_gfx_euler_zyx_degrees,
    };
}

std::vector<config::edit> world_object_window::settings_edits(const config::document &carried) const
{
    settings chosen      = state();
    chosen.selected_view = m_chosen_view;

    return config::unsaved_edits(carried, keys::held_as_carried(carried, write_world_object(chosen, m_settings_at)));
}

void world_object_window::render()
{
    ImGui::Begin(display_name().c_str());
    if(m_world_object != nullptr)
        render_activation();

    if(m_world_view == world_view::transform)
        render_graphics_transform();
    else
        render_stl_loader();
    ImGui::End();
}

// The starting state is the object the stencil was built with; the loader is reached only from the
// swap control below.
void world_object_window::initialize()
{
    place_world_object(m_stencil, m_frame, state());
    if(m_world_object == nullptr)
        m_world_view.set(world_view::load_stl);
    else if(m_active)
        m_world_view.set(world_view::transform);
}

void world_object_window::render_activation()
{
    if(ImGui::Checkbox("Active", &m_active))
    {
        if(m_active)
        {
            activate_loaded_object();
            m_world_view.set(world_view::transform);
        }
        else
        {
            deactivate_loaded_object();
            m_world_view.set(world_view::load_stl);
        }
        m_chosen_view = m_world_view.value();
    }
    ImGui::Text("Loaded model: %s", m_loaded.c_str());
}

void world_object_window::render_stl_loader()
{
    ImGui::InputText("STL file", m_model_path, sizeof(m_model_path));
    if(ImGui::Button("Load"))
    {
        m_active = load_stl();
        m_loaded = loaded_model_line(m_model_path, m_roots, m_active);
        if(m_active)
        {
            m_world_view.set(world_view::transform);
            m_chosen_view = m_world_view.value();
            activate_loaded_object();
        }
    }
    if(m_world_object == nullptr && !m_loaded.empty())
        ImGui::Text("Loaded model: %s", m_loaded.c_str());
}

void world_object_window::render_graphics_transform()
{
    const char *const scale_labels[3]{"SX", "SY", "SZ"};
    const char *const position_labels[3]{"X", "Y", "Z"};
    const char *const orientation_labels[3]{"A", "B", "C"};
    const auto reassign = [&](int) { assign_gfx_transform(); };
    scene::render_float3_inputs(m_gfx_offset, position_labels, 0.1f, 1.f, reassign);
    ImGui::NewLine();
    scene::render_float3_inputs_with_reset(m_gfx_euler_zyx_degrees, orientation_labels, 1.f, 10.f, reassign);
    ImGui::NewLine();
    scene::render_float3_inputs(m_gfx_scale, scale_labels, 0.1f, 1.f, reassign);
}

void world_object_window::activate_loaded_object()
{
    m_stencil.set_world_object(m_world_object);
    assign_gfx_transform();
}

void world_object_window::deactivate_loaded_object()
{
    m_stencil.clear_world_object();
}

void world_object_window::clear_loaded_object()
{
    deactivate_loaded_object();
    if(m_world_object)
        m_world_object.reset();
}

bool world_object_window::load_stl()
{
    clear_loaded_object();
    const std::optional<std::filesystem::path> file = located_model(m_model_path, m_roots);
    if(!file)
        return false;

    m_world_object = loaded_model(*file);

    return m_world_object != nullptr;
}

}
