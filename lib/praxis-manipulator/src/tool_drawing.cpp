#include "robot/chain_figure.h"
#include "robot/joint_decoration.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <threepp/math/Color.hpp>
#include <threepp/math/Matrix4.hpp>
#include <threepp/math/Vector3.hpp>

#include <threepp/materials/MeshPhongMaterial.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <cstddef>
#include <cstdint>

namespace praxis::manipulator {

namespace {

constexpr std::uint32_t tool_stick_tone = 0x404040;

Eigen::Vector3d position_of(const threepp::Matrix4 &placed)
{
    threepp::Vector3 at;
    at.setFromMatrixPosition(placed);

    return Eigen::Vector3d{at.x, at.y, at.z};
}

}

std::shared_ptr<threepp::Material> tool_stick_material()
{
    return threepp::MeshPhongMaterial::create({{"flatShading", true}, {"color", threepp::Color(tool_stick_tone)}});
}

void loadable_robot_stencil::set_tool_mesh_shown(bool shown)
{
    m_tool_mesh_shown = shown;
}

// The switch is the group's, because placing the segment writes the segment's own visibility.
void loadable_robot_stencil::set_tool_stick_shown(bool shown)
{
    m_tool_stick->visible = shown;
}

std::string loadable_robot_stencil::tool_stick_name()
{
    return "Tool stick";
}

// The mesh switch is applied every frame because a mesh can be attached under the tool key at any
// time.
void loadable_robot_stencil::place_tool_drawing() const
{
    const std::shared_ptr<threepp::Object3D> &mesh = m_attached[static_cast<std::size_t>(flange_attachment::tool)].object;
    if(mesh != nullptr)
        mesh->visible = m_tool_mesh_shown;

    const std::shared_ptr<const arm_snapshot> seen = m_seen.read();
    const threepp::Matrix4 flange                  = m_robot->getEndEffectorTransform();
    threepp::Matrix4 tool(flange);
    if(seen != nullptr)
        tool.multiply(to_renderer_transform(seen->tool_offset));

    place_segment(*m_tool_segment, position_of(flange), position_of(tool));
}

}
