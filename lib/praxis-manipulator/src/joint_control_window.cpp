#include "joint_naming.h"

#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/joint_control_window.h"
#include "praxis/manipulator/control_configuration.h"

#include "praxis/extension/held_handle.h"

#include "praxis/rigid_motion/angles.h"

#include <imgui.h>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::manipulator {

namespace {

constexpr std::array<control_mode, 2> mode_options{control_mode::preview, control_mode::simulation};

constexpr const char *unpublished_arm = "The arm has published nothing yet.";

}

joint_control_window::settings::settings(control_mode chosen)
        : mode(chosen)
{
}

joint_control_window::joint_control_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm)
        : joint_control_window(std::move(name), std::move(seen), std::move(arm), settings{})
{
}

joint_control_window::joint_control_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const settings &state, std::string at)
        : imgui_window(std::move(name))
        , m_seen(seen)
        , m_settings_at(std::move(at))
        , m_arm(std::move(arm))
        , m_control_mode(state.mode, mode_options, control_mode_labels())
        , m_control_widgets(control_widgets::fields, {control_widgets::sliders, control_widgets::fields}, {"Sliders", "Fields"})
{
    const std::shared_ptr<const arm_snapshot> share = seen.read();

    seed_fields(held(share, "the joint control window", "published arm state"));
}

joint_control_window::settings joint_control_window::state() const
{
    return settings{m_control_mode.value()};
}

std::vector<config::edit> joint_control_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_joint_control(state(), m_settings_at));
}

void joint_control_window::render()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();

    ImGui::Begin(display_name().c_str());
    if(published == nullptr)
        ImGui::TextUnformatted(unpublished_arm);
    else
    {
        render_option_cycle("Control mode", m_control_mode);
        render_option_cycle("Widgets", m_control_widgets);
        render_joint_space(*published);
    }
    ImGui::End();
}

bool joint_control_window::render_joint_inputs(const arm_snapshot &seen)
{
    const Eigen::Index shown = m_joint_positions.size();
    const bool bounded       = m_control_widgets == control_widgets::sliders && seen.limits.lower_position.size() >= shown && seen.limits.upper_position.size() >= shown;
    bool edited              = false;

    for(Eigen::Index joint = 0; joint < shown; ++joint)
    {
        const char *label = m_joint_labels[static_cast<std::size_t>(joint)].c_str();
        // The widget call is the left operand of the disjunction: the right one goes unevaluated
        // once a field has moved, and a field not drawn is a field not on screen.
        if(bounded)
            edited = ImGui::SliderFloat(label, &m_joint_positions[joint], static_cast<float>(to_degrees(seen.limits.lower_position[joint])),
                                        static_cast<float>(to_degrees(seen.limits.upper_position[joint]))) ||
                    edited;
        else
            edited = ImGui::InputFloat(label, &m_joint_positions[joint], 1.f, 10.f) || edited;
    }

    return edited;
}

void joint_control_window::render_joint_space(const arm_snapshot &seen)
{
    if(m_joint_positions.size() != seen.joints.size())
        seed_fields(seen);

    bool edited = render_joint_inputs(seen);
    if(ImGui::Button("Reset to zero"))
    {
        m_joint_positions = Eigen::VectorXf::Zero(m_joint_positions.size());
        edited            = true;
    }

    const joint_vector commanded = m_joint_positions.cast<double>() * radians_per_degree;
    if(ImGui::Button("Reset to current"))
        seed_fields(seen);

    render_joint_commands(commanded, edited);
}

void joint_control_window::render_joint_commands(const joint_vector &commanded, bool edited)
{
    if(m_control_mode == control_mode::preview)
    {
        if(edited)
            preview_configuration(commanded);

        return;
    }

    ImGui::SameLine();
    if(ImGui::Button("Move"))
        move_to_configuration(commanded);
}

void joint_control_window::seed_fields(const arm_snapshot &seen)
{
    m_joint_labels    = named_joints(static_cast<std::size_t>(seen.joints.size()));
    m_joint_positions = (seen.joints * degrees_per_radian).cast<float>();
}

void joint_control_window::preview_configuration(const joint_vector &commanded)
{
    command(m_arm, [commanded](robot_controller &control, scene_robot &) { control.preview_joint_configuration(commanded); });
}

void joint_control_window::move_to_configuration(const joint_vector &commanded)
{
    command(m_arm,
            [commanded](robot_controller &control, scene_robot &)
            {
                const std::array<joint_vector, 1> target{commanded};
                control.joint_space_trajectory(target);
            });
}

}
