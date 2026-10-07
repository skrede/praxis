#include "placement_reading.h"

#include "robot/joint_decoration.h"

#include "praxis/manipulator/model_file.h"
#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/model_placement.h"

#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/types.h"

#include <memory>
#include <utility>
#include <optional>
#include <filesystem>

namespace praxis::manipulator {

namespace {

transform read_transform(const rigid_motion::frame_ops &frames, const Eigen::Vector3f &euler_degrees, axis_order order, const Eigen::Vector3f &offset, placement_doubts &doubts)
{
    const rotation turned = read_rotation(frames, euler_degrees, order, doubts);
    const transform read  = frames.transformation_matrix_from_rotation_position(turned, offset.cast<double>());
    note_reading(frames, rigid_motion::frame_slot::transformation_matrix_from_rotation_position, turned.allFinite() && offset.allFinite(), read.allFinite(), doubts);

    return read;
}

void place_mesh(loadable_robot_stencil &on, threepp::Object3D &mesh, const rigid_motion::frame_ops &frames, const tool_window::settings &state, placement_doubts &doubts)
{
    const transform placed = read_transform(frames, state.gfx_euler_degrees, state.gfx_euler_order, state.gfx_offset, doubts);

    mesh.scale = threepp::Vector3(state.gfx_scale.x(), state.gfx_scale.y(), state.gfx_scale.z());
    on.set_flange_attachment_offset(flange_attachment::tool, to_renderer_transform(placed));
}

void post_offset(const std::weak_ptr<owned_arm> &arm, const rigid_motion::frame_ops &frames, const tool_window::settings &state, placement_doubts &doubts)
{
    const transform offset = read_transform(frames, state.kinematics_euler_degrees, state.kinematics_euler_order, state.kinematics_offset, doubts);

    command(arm, [offset](robot_controller &, scene_robot &driven) { driven.set_tool_offset(offset); });
}

// Nothing hangs at the tool key, and the pose the arm reports is the flange's own.
void take_off(loadable_robot_stencil &on, const std::weak_ptr<owned_arm> &arm)
{
    on.clear_flange_attachment(flange_attachment::tool);
    command(arm, [](robot_controller &, scene_robot &driven) { driven.set_tool_offset(transform::Identity()); });
}

}

placement_doubts seat_tool(loadable_robot_stencil &on, const std::weak_ptr<owned_arm> &arm, const rigid_motion::frame_ops &frames, const tool_window::settings &state)
{
    placement_doubts doubts;
    const std::shared_ptr<threepp::Object3D> mesh = on.attached_at(flange_attachment::tool);
    if(!state.active || mesh == nullptr)
    {
        take_off(on, arm);

        return doubts;
    }

    place_mesh(on, *mesh, frames, state, doubts);
    post_offset(arm, frames, state, doubts);

    return doubts;
}

void tool_window::assign_gfx_transform()
{
    if(m_tool == nullptr)
        return;

    placement_doubts unreported;
    place_mesh(m_stencil, *m_tool, m_frame, state(), unreported);
}

void tool_window::assign_kinematics_transform()
{
    placement_doubts unreported;
    post_offset(m_arm, m_frame, state(), unreported);
}

void tool_window::activate_custom_tool()
{
    m_stencil.set_flange_attachment(flange_attachment::tool, m_tool);
    assign_gfx_transform();
    assign_kinematics_transform();
}

void tool_window::activate_default_tool()
{
    take_off(m_stencil, m_arm);
}

bool tool_window::load_stl()
{
    activate_default_tool();
    m_tool.reset();
    const std::optional<std::filesystem::path> file = located_model(m_model_path, m_roots);
    if(!file)
        return false;

    std::shared_ptr<threepp::Object3D> loaded = loaded_model(*file);
    if(loaded == nullptr)
        return false;

    m_tool = std::move(loaded);

    return true;
}

}
