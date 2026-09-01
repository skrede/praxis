#include "praxis/manipulator/render_controls_window.h"

#include <imgui.h>

#include <array>
#include <string>
#include <utility>
#include <cstddef>
#include <optional>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// A body of no extent draws nothing while still reporting a length, so no field admits a value below
// this. In drawn metres.
constexpr float smallest_length = 0.001f;

// A cut of no extent draws nothing while still reporting a value, so no field admits a multiple
// below this. Dimensionless, as the cut is.
constexpr float smallest_ratio = 0.01f;

constexpr std::size_t angular = static_cast<std::size_t>(jacobian_block::angular);
constexpr std::size_t linear  = static_cast<std::size_t>(jacobian_block::linear);

float floored(const std::optional<double> &named, double opened, float least)
{
    return std::max(least, static_cast<float>(named.value_or(opened)));
}

}

render_controls_window::render_controls_window(std::string name, loadable_robot_stencil &drawn)
        : render_controls_window(std::move(name), drawn, controls(), settings{})
{
}

render_controls_window::render_controls_window(std::string name, loadable_robot_stencil &drawn, const controls &offered, const settings &state, std::string at)
        : imgui_window(std::move(name))
        , m_cap_named(state.force_cap_ratio.has_value())
        , m_scale_named{state.angular_scale.has_value(), state.linear_scale.has_value()}
        , m_column_named{state.angular_column_scale.has_value(), state.linear_column_scale.has_value()}
        , m_controls(offered)
        , m_settings_at(std::move(at))
        , m_drawn(drawn)
        , m_force_cap_ratio(floored(state.force_cap_ratio, drawn.force_cap_ratio(), smallest_ratio))
        , m_scale{floored(state.angular_scale, drawn.ellipsoid_scale(jacobian_block::angular), smallest_length),
                  floored(state.linear_scale, drawn.ellipsoid_scale(jacobian_block::linear), smallest_length)}
        , m_column_scale{floored(state.angular_column_scale, drawn.column_scale(jacobian_block::angular), smallest_length),
                         floored(state.linear_column_scale, drawn.column_scale(jacobian_block::linear), smallest_length)}
{
}

void render_controls_window::initialize()
{
    m_drawn.set_ellipsoid_scale(jacobian_block::angular, static_cast<double>(m_scale[angular]));
    m_drawn.set_ellipsoid_scale(jacobian_block::linear, static_cast<double>(m_scale[linear]));
    m_drawn.set_column_scale(jacobian_block::angular, static_cast<double>(m_column_scale[angular]));
    m_drawn.set_column_scale(jacobian_block::linear, static_cast<double>(m_column_scale[linear]));
    m_drawn.set_force_cap_ratio(static_cast<double>(m_force_cap_ratio));
}

void render_controls_window::render()
{
    ImGui::Begin(display_name().c_str());
    if(m_controls.ellipsoid_scale)
        render_lengths(true);
    if(m_controls.force_cap)
        render_cap();
    if(m_controls.column_scale)
        render_lengths(false);
    ImGui::End();
}

void render_controls_window::render_cap()
{
    if(!ImGui::InputFloat("Force cap (x scale)", &m_force_cap_ratio, 0.1f, 0.5f))
        return;

    m_force_cap_ratio = std::max(m_force_cap_ratio, smallest_ratio);
    m_cap_named       = true;
    m_drawn.set_force_cap_ratio(static_cast<double>(m_force_cap_ratio));
}

void render_controls_window::render_lengths(bool ellipsoids)
{
    const char *const labels[2][jacobian_block_count]{{"Angular column scale (m)", "Linear column scale (m)"}, {"Angular scale (m)", "Linear scale (m)"}};
    std::array<float, jacobian_block_count> &held = ellipsoids ? m_scale : m_column_scale;
    std::array<bool, jacobian_block_count> &named = ellipsoids ? m_scale_named : m_column_named;
    for(const jacobian_block which : {jacobian_block::angular, jacobian_block::linear})
    {
        const std::size_t at = static_cast<std::size_t>(which);
        if(!ImGui::InputFloat(labels[ellipsoids ? 1 : 0][at], &held[at], 0.01f, 0.05f))
            continue;

        held[at]  = std::max(held[at], smallest_length);
        named[at] = true;
        if(ellipsoids)
            m_drawn.set_ellipsoid_scale(which, static_cast<double>(held[at]));
        else
            m_drawn.set_column_scale(which, static_cast<double>(held[at]));
    }
}

}
