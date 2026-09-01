#include "robot/ellipsoid_figure.h"
#include "robot/ellipsoid_placement.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <threepp/objects/Group.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/constants.hpp>

#include <threepp/materials/LineBasicMaterial.hpp>
#include <threepp/materials/MeshPhongMaterial.hpp>

#include <Eigen/Core>

#include <cmath>
#include <string>
#include <memory>
#include <cstddef>
#include <optional>
#include <algorithm>

namespace praxis::manipulator {

namespace {

constexpr std::size_t ramp_steps   = 160;
constexpr double condition_ceiling = 1.0e4;

constexpr threepp::Color::ColorName well_conditioned_tone = threepp::Color::mediumblue;
constexpr threepp::Color::ColorName ill_conditioned_tone  = threepp::Color::gold;

threepp::Color ramp_tone(std::size_t step)
{
    const float along = ramp_steps < 2u ? 0.f : static_cast<float>(std::min(step, ramp_steps - 1u)) / static_cast<float>(ramp_steps - 1u);

    threepp::Color toned(well_conditioned_tone);

    return toned.lerpHSL(threepp::Color(ill_conditioned_tone), along);
}

std::size_t block_of(jacobian_block which)
{
    return static_cast<std::size_t>(which);
}

}

std::size_t ellipsoid_ramp_steps()
{
    return ramp_steps;
}

std::size_t ellipsoid_ramp_step(std::optional<double> condition)
{
    const std::size_t last = ramp_steps - 1u;
    if(!condition || !std::isfinite(*condition))
        return last;

    const double along = std::log10(*condition) / std::log10(condition_ceiling);

    return static_cast<std::size_t>(std::llround(std::clamp(along, 0.0, 1.0) * static_cast<double>(last)));
}

// A semi-axis of zero flattens the body onto a plane and a cap cuts it flat, so the surface is seen
// from either face.
std::shared_ptr<threepp::Material> ellipsoid_material(std::size_t step, bool wireframe)
{
    return threepp::MeshPhongMaterial::create({{"flatShading", true}, {"wireframe", wireframe}, {"side", threepp::Side::Double}, {"color", ramp_tone(step)}});
}

std::shared_ptr<threepp::Material> continuation_material(std::size_t step)
{
    return threepp::LineBasicMaterial::create({{"color", ramp_tone(step)}});
}

std::string loadable_robot_stencil::manipulability_ellipsoid_name(jacobian_block which)
{
    return which == jacobian_block::angular ? "Angular manipulability ellipsoid" : "Linear manipulability ellipsoid";
}

std::string loadable_robot_stencil::ellipsoid_continuation_name(jacobian_block which, std::size_t axis, bool forward)
{
    return manipulability_ellipsoid_name(which) + " axis " + std::to_string(axis + 1) + (forward ? " continuation along" : " continuation against");
}

void loadable_robot_stencil::set_jacobian_frame(jacobian_frame taken)
{
    m_frame = taken;
}

jacobian_frame loadable_robot_stencil::jacobian_frame_shown() const
{
    return m_frame;
}

void loadable_robot_stencil::set_ellipsoid_scale(jacobian_block which, double drawn_metres_per_unit)
{
    m_ellipsoid_scale[block_of(which)] = drawn_metres_per_unit;
}

double loadable_robot_stencil::ellipsoid_scale(jacobian_block which) const
{
    return m_ellipsoid_scale[block_of(which)];
}

void loadable_robot_stencil::set_force_cap_ratio(double of_the_ellipsoid_scale)
{
    m_force_cap_ratio = of_the_ellipsoid_scale;
}

double loadable_robot_stencil::force_cap_ratio() const
{
    return m_force_cap_ratio;
}

void loadable_robot_stencil::set_force_capped(bool capped)
{
    m_force_capped = capped;
}

bool loadable_robot_stencil::force_capped() const
{
    return m_force_capped;
}

void loadable_robot_stencil::set_manipulability_ellipsoids(ellipsoid_view read)
{
    m_view = read;
    raise_ellipsoid(jacobian_block::angular);
    raise_ellipsoid(jacobian_block::linear);
}

void loadable_robot_stencil::clear_manipulability_ellipsoids()
{
    for(std::size_t block = 0; block < jacobian_block_count; ++block)
    {
        m_ellipsoid_groups[block]->clear();
        m_ellipsoids[block] = drawn_ellipsoid{};
    }
}

void loadable_robot_stencil::set_angular_ellipsoid_shown(bool shown)
{
    m_ellipsoid_groups[block_of(jacobian_block::angular)]->visible = shown;
}

void loadable_robot_stencil::set_linear_ellipsoid_shown(bool shown)
{
    m_ellipsoid_groups[block_of(jacobian_block::linear)]->visible = shown;
}

// The angular block is drawn as a wireframe and the linear one solid; both wear the same
// condition-number ramp, so the surface treatment is what tells the two apart.
void loadable_robot_stencil::raise_ellipsoid(jacobian_block which)
{
    drawn_ellipsoid &raised = m_ellipsoids[block_of(which)];
    if(raised.body != nullptr)
        return;

    const std::shared_ptr<threepp::Object3D> &under = m_ellipsoid_groups[block_of(which)];
    const bool wireframe                            = which == jacobian_block::angular;

    raised.body = ellipsoid_object(manipulability_ellipsoid_name(which), (wireframe ? m_ellipsoid_wire : m_ellipsoid_solid).front());
    under->add(raised.body);
    for(std::size_t line = 0; line < raised.lines.size(); ++line)
    {
        raised.lines[line] = continuation_object(ellipsoid_continuation_name(which, line / 2u, line % 2u == 0u), m_continuation_tone.front());
        under->add(raised.lines[line]);
    }
}

void loadable_robot_stencil::place_ellipsoids() const
{
    const std::shared_ptr<const arm_snapshot> seen = m_seen.read();
    if(seen == nullptr)
        return;

    const jacobian_manipulability &taken = m_frame == jacobian_frame::space ? seen->space_manipulability : seen->body_manipulability;
    for(std::size_t block = 0; block < jacobian_block_count; ++block)
    {
        const drawn_ellipsoid &standing = m_ellipsoids[block];
        if(standing.body == nullptr)
            continue;

        if(!seen->tool_position)
        {
            hide_ellipsoid_block(*standing.body, standing.lines);
            continue;
        }

        const bool angular              = block == block_of(jacobian_block::angular);
        const std::optional<double> cap = m_view == ellipsoid_view::force && m_force_capped ? std::optional<double>(m_force_cap_ratio * m_ellipsoid_scale[block]) : std::nullopt;
        const ellipsoid_tones tone{angular ? m_ellipsoid_wire : m_ellipsoid_solid, m_continuation_tone};
        place_ellipsoid_block(angular ? taken.angular : taken.linear, *seen->tool_position, m_view, m_ellipsoid_scale[block], cap, *standing.body, standing.lines, tone);
    }
}

}
