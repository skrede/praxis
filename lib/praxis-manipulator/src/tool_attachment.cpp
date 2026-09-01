#include "praxis/manipulator/tool_window.h"

#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/angles.h"

#include <array>
#include <string>
#include <cstddef>

namespace praxis::manipulator {

namespace {

// The renderer stores a transform column by column, and in single precision.
threepp::Matrix4 to_renderer_transform(const transform &tf)
{
    std::array<float, 16> rendered{};
    for(Eigen::Index column = 0; column < 4; ++column)
        for(Eigen::Index row = 0; row < 4; ++row)
            rendered[static_cast<std::size_t>(4 * column + row)] = static_cast<float>(tf(row, column));

    return threepp::Matrix4(rendered);
}

}

void tool_window::assign_gfx_transform()
{
    if(m_tool == nullptr)
        return;

    const Eigen::Vector3d angles = m_gfx_euler_degrees.cast<double>() * radians_per_degree;
    const transform tf           = m_frame.transformation_matrix_from_rotation_position(m_frame.rotation_matrix_from_euler(angles, m_gfx_euler_order), m_gfx_offset.cast<double>());

    m_tool->scale = threepp::Vector3(m_gfx_scale.x(), m_gfx_scale.y(), m_gfx_scale.z());
    m_stencil.set_flange_attachment_offset(flange_attachment::tool, to_renderer_transform(tf));
}

void tool_window::assign_kinematics_transform()
{
    const Eigen::Vector3d angles = m_tool_euler_degrees.cast<double>() * radians_per_degree;
    const transform offset       = m_frame.transformation_matrix_from_rotation_position(m_frame.rotation_matrix_from_euler(angles, m_tool_euler_order), m_tool_offset.cast<double>());

    command(m_arm, [offset](robot_controller &, scene_robot &driven) { driven.set_tool_offset(offset); });
}

void tool_window::activate_custom_tool()
{
    m_stencil.set_flange_attachment(flange_attachment::tool, m_tool);
    assign_gfx_transform();
    assign_kinematics_transform();
}

// No tool attached: nothing hangs at the tool key, and the arm's tool offset returns to the identity
// so that the pose the arm reports is the flange's own.
void tool_window::activate_default_tool()
{
    m_stencil.clear_flange_attachment(flange_attachment::tool);
    command(m_arm, [](robot_controller &, scene_robot &driven) { driven.set_tool_offset(transform::Identity()); });
}

bool tool_window::load_stl()
{
    threepp::STLLoader loader;
    const std::string path(m_model_path);
    if(path.empty())
        return false;

    const auto geometry = loader.load(path);
    if(geometry == nullptr)
        return false;

    m_tool = threepp::Mesh::create(geometry, threepp::MeshPhongMaterial::create({{"flatShading", true}, {"color", threepp::Color::gray}}));

    return true;
}

}
