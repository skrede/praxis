#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/control_configuration.h"
#include "praxis/manipulator/control_parameters_window.h"

#include "praxis/extension/held_handle.h"

#include "praxis/rigid_motion/angles.h"

#include <imgui.h>

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <utility>
#include <optional>
#include <functional>

namespace praxis::manipulator {

namespace {

constexpr const char *unpublished_arm = "The arm has published nothing yet.";

float published_velocity_factor(const arm_reader &seen)
{
    return static_cast<float>(held(seen.read(), "the control parameters window", "published arm state").velocity_factor);
}

float rate_of(const std::optional<path_parameter_bounds> &held_to)
{
    return held_to ? static_cast<float>(held_to->max_rate) : 0.f;
}

float rate_change_of(const std::optional<path_parameter_bounds> &held_to)
{
    return held_to ? static_cast<float>(held_to->max_rate_change) : 0.f;
}

}

control_parameters_window::settings::settings(float chosen_velocity, time_scaling_choice chosen_scaling, std::optional<path_parameter_bounds> chosen_trapezoid)
        : velocity(chosen_velocity)
        , scaling(chosen_scaling)
        , trapezoid(chosen_trapezoid)
{
}

control_parameters_window::control_parameters_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, std::function<void()> changed)
        : control_parameters_window(std::move(name), seen, std::move(arm), settings{published_velocity_factor(seen)}, std::string(), std::move(changed))
{
}

control_parameters_window::control_parameters_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const settings &state, std::string at,
                                                     std::function<void()> changed)
        : imgui_window(std::move(name))
        , m_velocity(state.velocity)
        , m_max_rate(rate_of(state.trapezoid))
        , m_overridden(state.trapezoid.has_value())
        , m_seen(std::move(seen))
        , m_max_rate_change(rate_change_of(state.trapezoid))
        , m_settings_at(std::move(at))
        , m_arm(std::move(arm))
        , m_changed_cb(std::move(changed))
        , m_scaling(state.scaling, time_scaling_options, time_scaling_labels)
{
}

control_parameters_window::settings control_parameters_window::state() const
{
    return settings{m_velocity, m_scaling.value(), held_bounds()};
}

std::vector<config::edit> control_parameters_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_control_parameters(state(), m_settings_at));
}

void control_parameters_window::render()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();

    ImGui::Begin(display_name().c_str());
    if(published == nullptr)
        ImGui::TextUnformatted(unpublished_arm);
    else
    {
        render_playback_rate();
        render_time_scaling();
        render_trapezoid_bounds(*published);
        render_joint_bounds(*published);
    }
    ImGui::End();
}

std::optional<path_parameter_bounds> control_parameters_window::held_bounds() const
{
    if(!m_overridden || !(m_max_rate > 0.f) || !(m_max_rate_change > 0.f))
        return std::nullopt;

    return path_parameter_bounds{static_cast<double>(m_max_rate), static_cast<double>(m_max_rate_change)};
}

void control_parameters_window::render_playback_rate()
{
    if(!ImGui::SliderFloat("Velocity factor", &m_velocity, 0.f, 1.f))
        return;

    const double factor = static_cast<double>(m_velocity);
    command(m_arm, [factor](robot_controller &control, scene_robot &) { control.set_velocity_factor(factor); });
}

void control_parameters_window::render_time_scaling()
{
    if(!render_option_cycle("Time scaling", m_scaling))
        return;

    const time_scaling_choice chosen = m_scaling.value();
    command(m_arm, [chosen](robot_controller &control, scene_robot &) { control.set_time_scaling(chosen); });
    if(m_changed_cb)
        m_changed_cb();
}

void control_parameters_window::render_trapezoid_bounds(const arm_snapshot &seen)
{
    bool changed = ImGui::Checkbox("Override trapezoid bounds", &m_overridden);
    if(changed && m_overridden && !(m_max_rate > 0.f))
    {
        const path_parameter_bounds opened = derived_bounds(seen.limits, seen.limits.lower_position, seen.limits.upper_position);
        m_max_rate                         = static_cast<float>(opened.max_rate);
        m_max_rate_change                  = static_cast<float>(opened.max_rate_change);
    }
    if(m_overridden)
    {
        changed = ImGui::InputFloat("Path rate", &m_max_rate) || changed;
        changed = ImGui::InputFloat("Path rate change", &m_max_rate_change) || changed;
    }
    if(!changed)
        return;

    const std::optional<path_parameter_bounds> held = held_bounds();
    command(m_arm, [held](robot_controller &control, scene_robot &) { control.set_trapezoid_bounds(held); });
    if(m_changed_cb)
        m_changed_cb();
}

void control_parameters_window::render_joint_bounds(const arm_snapshot &seen)
{
    const std::uint32_t joints = static_cast<std::uint32_t>(seen.joints.size());
    for(std::uint32_t joint = 0u; joint < joints; ++joint)
    {
        const std::string name = "J" + std::to_string(joint + 1u);
        ImGui::Value((name + " v_max").c_str(), static_cast<float>(to_degrees(seen.limits.velocity[joint])) * m_velocity);
        ImGui::Value((name + " a_max").c_str(), static_cast<float>(to_degrees(seen.limits.acceleration[joint])) * m_velocity);
        if(joint + 1u != joints)
            ImGui::Separator();
    }
}

}
