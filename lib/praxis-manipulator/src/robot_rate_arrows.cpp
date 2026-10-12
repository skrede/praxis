#include "robot/column_arrow.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <spdlog/spdlog.h>

#include <threepp/math/Color.hpp>

#include <threepp/materials/MeshPhongMaterial.hpp>

#include <Eigen/Core>

#include <span>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// sRGB hex, as threepp::Color reads a hex.
constexpr std::uint32_t angular_twist_tone = 0xff8c00;
constexpr std::uint32_t linear_twist_tone  = 0x9b7700;

refusal rates_of_another_width(std::size_t told, std::size_t columns)
{
    spdlog::error("praxis: {} joint rates were told to a drawing of {} columns, so no column is weighted by them and the rates standing are left as they were", told, columns);

    return refusal::unsupported_input;
}

refusal rate_not_finite()
{
    spdlog::error("praxis: a joint rate told to the drawing is not a finite number, so no column is weighted by the rates told and the rates standing are left as they were");

    return refusal::unsupported_input;
}

// The twist the rates give, angular part then linear part, each carried into the space frame.
std::array<Eigen::Vector3d, jacobian_block_count> drawn_twist(const jacobian &taken, const std::vector<double> &rates, const rotation &into_space, jacobian_frame frame,
                                                              const Eigen::Vector3d &tool)
{
    const Eigen::Matrix<double, 6, 1> twist = taken * Eigen::Map<const Eigen::VectorXd>(rates.data(), static_cast<Eigen::Index>(rates.size()));
    const Eigen::Vector3d spin              = into_space * twist.head<3>();
    Eigen::Vector3d moving                  = into_space * twist.tail<3>();
    // The space twist's linear part is the velocity of the point at the space origin; the tool point's
    // adds the angular part crossed into the tool position. Lynch & Park, Modern Robotics, section 5.1.
    if(frame == jacobian_frame::space)
        moving += spin.cross(tool);

    return {spin, moving};
}

}

threepp::Color twist_tone(jacobian_block part)
{
    return threepp::Color(part == jacobian_block::angular ? angular_twist_tone : linear_twist_tone);
}

std::shared_ptr<threepp::Material> twist_material(jacobian_block part)
{
    return threepp::MeshPhongMaterial::create({{"flatShading", true}, {"color", twist_tone(part)}});
}

std::string loadable_robot_stencil::twist_name(jacobian_block part)
{
    return part == jacobian_block::angular ? "Twist angular part" : "Twist linear part";
}

expected<void, refusal> loadable_robot_stencil::set_joint_rates(std::span<const double> rates)
{
    if(rates.empty())
    {
        m_rates.clear();
        return {};
    }
    if(rates.size() != m_column_arrows.size())
        return unexpected(rates_of_another_width(rates.size(), m_column_arrows.size()));
    if(!std::ranges::all_of(rates, [](double rate) { return std::isfinite(rate); }))
        return unexpected(rate_not_finite());

    m_rates.assign(rates.begin(), rates.end());

    return {};
}

void loadable_robot_stencil::set_columns_weighted(bool weighted)
{
    m_weighted = weighted;
}

bool loadable_robot_stencil::columns_weighted() const
{
    return m_weighted;
}

void loadable_robot_stencil::set_twist_shown(bool shown)
{
    m_twist->visible = shown;
}

bool loadable_robot_stencil::twist_shown() const
{
    return m_twist->visible;
}

void loadable_robot_stencil::raise_twist()
{
    for(std::size_t part = 0; part < jacobian_block_count; ++part)
    {
        const jacobian_block which = static_cast<jacobian_block>(part);
        const drawn_arrow raised   = arrow_object(twist_name(which), twist_material(which));

        m_twist_arrows[part] = drawn_column{raised.object, raised.shaft, raised.tip};
        m_twist_parts[part]->add(raised.object);
        m_twist->add(m_twist_parts[part]);
    }
    m_twist->visible = false;
}

void loadable_robot_stencil::hide_twist() const
{
    for(const drawn_column &part : m_twist_arrows)
        part.object->visible = false;
}

void loadable_robot_stencil::place_twist() const
{
    const std::shared_ptr<const arm_snapshot> seen = m_seen.read();
    if(seen == nullptr || m_rates.empty())
    {
        hide_twist();
        return;
    }

    const expected<jacobian, refusal> &taken     = shown_jacobian(*seen, m_frame);
    const expected<rotation, refusal> into_space = carried_into_space(*seen);
    if(!taken || !into_space || !seen->tool_position || static_cast<std::size_t>(taken->cols()) != m_rates.size())
    {
        hide_twist();
        return;
    }

    const std::array<Eigen::Vector3d, jacobian_block_count> along = drawn_twist(*taken, m_rates, *into_space, m_frame, *seen->tool_position);
    for(std::size_t part = 0; part < jacobian_block_count; ++part)
    {
        const drawn_column &standing = m_twist_arrows[part];
        place_arrow(drawn_arrow{standing.object, standing.shaft, standing.tip}, *seen->tool_position, along[part], along[part].norm() * m_column_scale[part]);
    }
}

}
