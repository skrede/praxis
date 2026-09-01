#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/screw_jog_window.h"
#include "praxis/manipulator/control_configuration.h"

#include "praxis/extension/held_handle.h"

#include "praxis/scene/widgets.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/axis_order.h"
#include "praxis/rigid_motion/screw_widgets.h"

#include <imgui.h>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <optional>

namespace praxis::manipulator {

namespace {

constexpr std::array<control_mode, 2> mode_options{control_mode::preview, control_mode::simulation};

constexpr const char *unpublished_arm = "The arm has published nothing yet.";

// The direction is three fields a reader edits one at a time, so it passes through zero on the way
// from one axis to another. A zero direction names no axis and the resolution refuses it, so the
// half-typed value is not carried across the seam at all.
std::optional<Eigen::Vector3d> named_axis(const Eigen::Vector3f &direction)
{
    const Eigen::Vector3d named = direction.cast<double>();
    if(named.isZero())
        return std::nullopt;

    return named.normalized();
}

}

screw_jog_window::settings::settings(control_mode chosen)
        : mode(chosen)
{
}

screw_jog_window::screw_jog_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited)
        : screw_jog_window(std::move(name), std::move(seen), std::move(arm), injected, std::move(edited), settings{})
{
}

screw_jog_window::screw_jog_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited,
                                   const settings &state, std::string at)
        : imgui_window(std::move(name))
        , m_pitch(0.f)
        , m_seen(seen)
        , m_q(Eigen::Vector3f::Zero())
        , m_w({0.f, 0.f, 1.f})
        , m_theta_degrees(0.f)
        , m_settings_at(std::move(at))
        , m_arm(std::move(arm))
        , m_frame(injected)
        , m_edited(std::move(edited))
        , m_control_mode(state.mode, mode_options, control_mode_labels())
{
    static_cast<void>(held(seen.read(), "the screw jog window", "published arm state"));
    static_cast<void>(held(m_edited, "the screw jog window", "shared pose"));
}

screw_jog_window::settings screw_jog_window::state() const
{
    return settings{m_control_mode.value()};
}

std::vector<config::edit> screw_jog_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_screw_jog(state(), m_settings_at));
}

void screw_jog_window::render()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();

    ImGui::Begin(display_name().c_str());
    if(published == nullptr)
        ImGui::TextUnformatted(unpublished_arm);
    else
    {
        render_option_cycle("Control mode", m_control_mode);
        render_screw_jog(*published);
    }
    ImGui::End();
}

void screw_jog_window::clear_screw()
{
    m_q             = Eigen::Vector3f::Zero();
    m_w             = Eigen::Vector3f{0.f, 0.f, 1.f};
    m_theta_degrees = 0.f;
    m_pitch         = 0.f;
}

bool screw_jog_window::render_screw_axis()
{
    return rigid_motion::render_screw_inputs(m_q, m_w, m_pitch);
}

bool screw_jog_window::render_screw_start_pose(const arm_snapshot &seen)
{
    bool moved = ImGui::InputFloat3("XYZ", m_edited->position.data());
    moved      = ImGui::InputFloat3("ABC", m_edited->euler_degrees.data()) || moved;
    moved      = scene::render_enum_selection("Euler order", m_edited->order, axis_order_labels()) || moved;
    // The seeding re-derives the start pose from the arm and returns the screw to the identity, so
    // the motion it leaves composed puts the arm where it already stands.
    if(ImGui::Button("Reset start") && seed_from(*m_edited, seen, m_frame))
        clear_screw();

    return moved;
}

void screw_jog_window::render_screw_jog(const arm_snapshot &seen)
{
    bool moved = render_screw_start_pose(seen);
    moved      = render_screw_axis() || moved;
    moved      = ImGui::SliderFloat("theta", &m_theta_degrees, -180.f, 180.f) || moved;
    if(moved && m_control_mode == control_mode::preview)
        preview_screw();
    if(ImGui::Button("Reset"))
    {
        clear_screw();
        if(m_control_mode == control_mode::preview)
            preview_screw();
    }
    if(m_control_mode != control_mode::preview)
    {
        ImGui::SameLine();
        const std::optional<Eigen::Vector3d> axis = named_axis(m_w);
        if(ImGui::Button("Move") && axis)
            move_along_screw(*axis);
    }
}

void screw_jog_window::preview_screw()
{
    const std::optional<Eigen::Vector3d> axis = named_axis(m_w);
    if(!axis)
        return;

    const transform start        = pose_matrix(*m_edited, m_frame);
    const Eigen::Vector3d turned = *axis;
    const Eigen::Vector3d at     = m_q.cast<double>();
    const double angle           = to_radians(static_cast<double>(m_theta_degrees));
    const double pitch           = static_cast<double>(m_pitch);
    command(m_arm, [start, turned, at, angle, pitch](robot_controller &control, scene_robot &) { control.preview_task_space_screw(start, turned, at, angle, pitch); });
}

void screw_jog_window::move_along_screw(const Eigen::Vector3d &axis)
{
    const Eigen::Vector3d at = m_q.cast<double>();
    const double angle       = to_radians(static_cast<double>(m_theta_degrees));
    const double pitch       = static_cast<double>(m_pitch);

    command(m_arm, [axis, at, angle, pitch](robot_controller &control, scene_robot &) { control.task_space_screw(axis, at, angle, pitch); });
}

}
