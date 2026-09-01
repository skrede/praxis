#include "robot/column_arrow.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <spdlog/spdlog.h>

#include <threepp/math/Color.hpp>

#include <threepp/materials/MeshPhongMaterial.hpp>

#include <Eigen/Core>

#include <array>
#include <string>
#include <memory>
#include <cstddef>

namespace praxis::manipulator {

namespace {

constexpr threepp::Color::ColorName angular_part_tone = threepp::Color::plum;
constexpr threepp::Color::ColorName linear_part_tone  = threepp::Color::deeppink;

std::size_t block_of(jacobian_block which)
{
    return static_cast<std::size_t>(which);
}

// The one field says which of the two the whole stencil is showing, so the columns cannot be taken
// from a matrix the ellipsoids beside them are not taken from.
const expected<jacobian, refusal> &shown_jacobian(const arm_snapshot &seen, jacobian_frame taken)
{
    if(taken == jacobian_frame::space)
        return seen.space_jacobian;

    return seen.body_jacobian;
}

refusal no_column_to_draw()
{
    spdlog::error("praxis: the columns of a Jacobian were told to stand at a width of none, so no arrow is raised and the ones standing are left as they were");

    return refusal::unsupported_input;
}

}

threepp::Color column_tone(jacobian_block part)
{
    return threepp::Color(part == jacobian_block::angular ? angular_part_tone : linear_part_tone);
}

std::shared_ptr<threepp::Material> column_material(jacobian_block part)
{
    return threepp::MeshPhongMaterial::create({{"flatShading", true}, {"color", column_tone(part)}});
}

std::string loadable_robot_stencil::jacobian_column_name(std::size_t column, jacobian_block part)
{
    return "Jacobian column " + std::to_string(column + 1) + (part == jacobian_block::angular ? " angular part" : " linear part");
}

void loadable_robot_stencil::set_column_scale(jacobian_block which, double drawn_metres_per_unit)
{
    m_column_scale[block_of(which)] = drawn_metres_per_unit;
}

double loadable_robot_stencil::column_scale(jacobian_block which) const
{
    return m_column_scale[block_of(which)];
}

// The width is taken before anything standing is removed, so a width the drawing cannot stand at
// leaves the arrows already standing where they were.
expected<void, refusal> loadable_robot_stencil::set_jacobian_columns(std::size_t columns)
{
    if(columns == 0u)
        return unexpected(no_column_to_draw());

    clear_jacobian_columns();
    m_column_arrows.resize(columns);
    for(std::size_t column = 0; column < columns; ++column)
        for(std::size_t part = 0; part < jacobian_block_count; ++part)
        {
            const drawn_arrow raised = arrow_object(jacobian_column_name(column, static_cast<jacobian_block>(part)), m_column_tone[part]);

            m_column_arrows[column][part] = drawn_column{raised.object, raised.shaft, raised.tip};
            m_columns->add(raised.object);
        }

    return {};
}

void loadable_robot_stencil::clear_jacobian_columns()
{
    m_columns->clear();
    m_column_arrows.clear();
}

void loadable_robot_stencil::set_jacobian_columns_shown(bool shown)
{
    m_columns->visible = shown;
}

void loadable_robot_stencil::hide_jacobian_columns() const
{
    for(const std::array<drawn_column, jacobian_block_count> &standing : m_column_arrows)
        for(const drawn_column &part : standing)
            part.object->visible = false;
}

// The linear part of a space Jacobian's column is the velocity of the material point currently at
// the space origin, and the linear part of a body Jacobian's column is the tool's own velocity, so
// each column stands at the point its linear part is the velocity of. Lynch & Park, Modern
// Robotics, section 5.1.
void loadable_robot_stencil::place_jacobian_columns() const
{
    const std::shared_ptr<const arm_snapshot> seen = m_seen.read();
    if(m_column_arrows.empty() || seen == nullptr)
        return;

    const expected<jacobian, refusal> &taken     = shown_jacobian(*seen, m_frame);
    const expected<Eigen::Vector3d, refusal> put = m_frame == jacobian_frame::space ? expected<Eigen::Vector3d, refusal>(Eigen::Vector3d::Zero()) : seen->tool_position;
    if(!taken || !put || static_cast<std::size_t>(taken->cols()) != m_column_arrows.size())
    {
        hide_jacobian_columns();
        return;
    }

    for(std::size_t column = 0; column < m_column_arrows.size(); ++column)
        for(std::size_t part = 0; part < jacobian_block_count; ++part)
        {
            const drawn_column &standing = m_column_arrows[column][part];
            const Eigen::Vector3d along  = taken->block<3, 1>(part == block_of(jacobian_block::angular) ? 0 : 3, static_cast<Eigen::Index>(column));

            place_arrow(drawn_arrow{standing.object, standing.shaft, standing.tip}, *put, along, along.norm() * m_column_scale[part]);
        }
}

}
