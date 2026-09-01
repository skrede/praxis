#include "praxis/manipulator/render_configuration.h"
#include "praxis/manipulator/render_controls_window.h"

#include <cstddef>
#include <optional>

namespace praxis::manipulator {

namespace {

constexpr std::size_t angular = static_cast<std::size_t>(jacobian_block::angular);
constexpr std::size_t linear  = static_cast<std::size_t>(jacobian_block::linear);

std::optional<double> chosen(bool named, float held)
{
    return named ? std::optional<double>(held) : std::nullopt;
}

}

render_controls_window::settings render_controls_window::state() const
{
    return settings{.angular_scale        = chosen(m_scale_named[angular], m_scale[angular]),
                    .linear_scale         = chosen(m_scale_named[linear], m_scale[linear]),
                    .angular_column_scale = chosen(m_column_named[angular], m_column_scale[angular]),
                    .linear_column_scale  = chosen(m_column_named[linear], m_column_scale[linear]),
                    .force_cap_ratio      = chosen(m_cap_named, m_force_cap_ratio)};
}

std::vector<config::edit> render_controls_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_render_controls(state(), m_settings_at));
}

}
