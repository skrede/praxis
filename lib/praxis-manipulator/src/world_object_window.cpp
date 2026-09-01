#include "praxis/manipulator/tool_configuration.h"
#include "praxis/manipulator/world_object_window.h"

#include "praxis/scene/widgets.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/axis_order.h"

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <cstring>
#include <utility>

namespace praxis::manipulator {

namespace {

// The renderer stores a transform column by column, and in single precision. Only the rotation
// block is read back from it: the object's position and scale are set on the node itself.
threepp::Matrix4 to_renderer_rotation(const rotation &r)
{
    std::array<float, 16> rendered{0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f};
    for(Eigen::Index column = 0; column < 3; ++column)
        for(Eigen::Index row = 0; row < 3; ++row)
            rendered[static_cast<std::size_t>(4 * column + row)] = static_cast<float>(r(row, column));

    return threepp::Matrix4(rendered);
}

}

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

world_object_window::world_object_window(std::string name, loadable_robot_stencil &target, const rigid_motion::frame_ops &injected, const settings &state, std::string at)
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

    return config::unsaved_edits(carried, write_world_object(chosen, m_settings_at));
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
    if(m_world_object == nullptr)
    {
        m_world_view.set(world_view::load_stl);
        return;
    }

    if(m_active)
    {
        assign_gfx_transform();
        m_world_view.set(world_view::transform);
    }
    else
        deactivate_loaded_object();
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
    ImGui::Text("Loaded model: %s", m_model_path);
}

void world_object_window::render_stl_loader()
{
    ImGui::InputText("STL file", m_model_path, sizeof(m_model_path));
    if(ImGui::Button("Load"))
    {
        m_active = load_stl();
        if(m_active)
        {
            m_world_view.set(world_view::transform);
            m_chosen_view = m_world_view.value();
            activate_loaded_object();
        }
    }
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

void world_object_window::assign_gfx_transform()
{
    if(m_world_object == nullptr)
        return;

    const Eigen::Vector3d angles = m_gfx_euler_zyx_degrees.cast<double>() * radians_per_degree;
    const rotation orientation   = m_frame.rotation_matrix_from_euler(angles, axis_order::zyx);

    m_world_object->scale    = threepp::Vector3(m_gfx_scale.x(), m_gfx_scale.y(), m_gfx_scale.z());
    m_world_object->position = threepp::Vector3(m_gfx_offset.x(), m_gfx_offset.y(), m_gfx_offset.z());
    m_world_object->setRotationFromMatrix(to_renderer_rotation(orientation));
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
    threepp::STLLoader loader;
    const std::string path(m_model_path);
    if(path.empty())
        return false;

    const auto geometry = loader.load(path);
    if(geometry == nullptr)
        return false;

    m_world_object = threepp::Mesh::create(geometry, threepp::MeshPhongMaterial::create({{"flatShading", true}, {"color", threepp::Color::gray}}));

    return true;
}

}
