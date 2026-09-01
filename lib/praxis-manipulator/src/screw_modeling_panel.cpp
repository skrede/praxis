#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/screw_modeling_window.h"

#include "praxis/evaluation/comparators.h"

#include "praxis/scene/widgets.h"

#include "praxis/rigid_motion/screw_widgets.h"

#include <imgui.h>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <vector>
#include <cstddef>

namespace praxis::manipulator {

namespace {

constexpr float length_step      = 0.01f;
constexpr float length_step_fast = 0.1f;
constexpr float angle_step       = 1.f;
constexpr float angle_step_fast  = 15.f;

constexpr std::array<screw_modeling_window::parameterization, 2> parameterization_options{screw_modeling_window::parameterization::point_direction_pitch,
                                                                                          screw_modeling_window::parameterization::angular_linear};

constexpr std::array<const char *, 2> parameterization_labels{"Point, direction, pitch", "Angular and linear"};

constexpr const char *const position_labels[3]{"X", "Y", "Z"};

// A value box prints three digits after the point, so half of the last of them is the smallest
// difference between two points a row is able to show.
constexpr float shown_point_tolerance = 0.0005f;

}

screw_modeling_window::row::row(parameterization opening)
        : pitch(0.f)
        , point(Eigen::Vector3f::Zero())
        , linear(Eigen::Vector3f::Zero())
        , angular(Eigen::Vector3f::Zero())
        , direction(Eigen::Vector3f::UnitZ())
        , typed(opening, parameterization_options, parameterization_labels)
{
}

void screw_modeling_window::row::show(const screw_axis &screw)
{
    angular = screw.head<3>().cast<float>();
    linear  = screw.tail<3>().cast<float>();
    if(screw.head<3>().norm() <= angular_epsilon)
        return;

    const Eigen::Vector3d along  = screw.head<3>().normalized();
    const Eigen::Vector3d scaled = screw.tail<3>() / screw.head<3>().norm();

    pitch     = static_cast<float>(along.dot(scaled));
    point     = along.cross(scaled).cast<float>();
    direction = along.cast<float>();
}

Eigen::Vector3f screw_modeling_window::stored_point(std::size_t joint) const
{
    row carried = m_rows[joint];
    carried.show(m_screws[joint]);

    return carried.point;
}

bool screw_modeling_window::shows_stored_point(std::size_t joint) const
{
    if(joint >= m_rows.size() || m_rows[joint].typed == parameterization::angular_linear)
        return true;

    return (stored_point(joint) - m_rows[joint].point).cwiseAbs().maxCoeff() < shown_point_tolerance;
}

void screw_modeling_window::render()
{
    ImGui::Begin(display_name().c_str());
    ImGui::TextUnformatted("These values describe the chain at its home configuration; the drawing shows the pose the arm holds.");
    render_home();
    render_selector();
    if(m_selected < m_rows.size())
        render_row(m_selected);

    ImGui::Separator();
    render_reading();
    render_save();
    ImGui::End();
}

// An entry's place in the list is the joint's own index, which is what lets the selection index the
// rows and the screws directly.
void screw_modeling_window::render_selector()
{
    if(scene::render_dropdown_selection("Joint", m_selected, m_entries))
        tell_selection();
}

void screw_modeling_window::render_home()
{
    bool edited          = false;
    const auto note_edit = [&edited](int) { edited = true; };

    ImGui::TextUnformatted("Home pose");
    ImGui::PushID("home");
    if(m_controls.home)
    {
        scene::render_float3_inputs(m_home_position, position_labels, length_step, length_step_fast, note_edit);
        scene::render_float3_inputs(m_home_euler_degrees, euler_input_labels, angle_step, angle_step_fast, note_edit);
    }
    else
        ImGui::Text("%.3f %.3f %.3f m", static_cast<double>(m_home_position.x()), static_cast<double>(m_home_position.y()), static_cast<double>(m_home_position.z()));
    ImGui::PopID();

    if(m_controls.reset && ImGui::Button("Reset chain"))
        reset();
    if(edited)
        assemble_home();
}

// The selector is a choice of input rather than a value: taking a different entry leaves the screw
// the row already carries exactly where it was, so only the numbers rebuild it.
void screw_modeling_window::render_row(std::size_t joint)
{
    row &shown  = m_rows[joint];
    bool edited = false;

    ImGui::PushID(static_cast<int>(joint));
    static_cast<void>(render_option_cycle("Typed as", shown.typed));
    if(shown.typed == parameterization::point_direction_pitch)
        edited = rigid_motion::render_screw_inputs(shown.point, shown.direction, shown.pitch, [this, joint] { return render_canonicalize(joint); });
    else
        edited = rigid_motion::render_twist_inputs(shown.angular, shown.linear);
    ImGui::PopID();

    if(edited)
        rebuild_row(joint);
}

bool screw_modeling_window::render_canonicalize(std::size_t joint)
{
    const bool pressed = ImGui::Button("Canonicalize");
    if(pressed)
        canonicalize(joint);
    if(shows_stored_point(joint))
        return pressed;

    const Eigen::Vector3f carried = stored_point(joint);
    ImGui::SameLine();
    ImGui::Text("%.3f %.3f %.3f", static_cast<double>(carried.x()), static_cast<double>(carried.y()), static_cast<double>(carried.z()));

    return pressed;
}

scene::readout screw_modeling_window::reading() const
{
    const std::shared_ptr<const arm_snapshot> share = m_seen.read();
    if(!share)
        return scene::readout{"The arm has published nothing yet.", {}};

    const expected<transform, refusal> supplied  = m_kinematics.forward_kinematics(m_home, m_screws, share->joints);
    const expected<transform, refusal> described = m_kinematics.forward_kinematics(m_derived.home, m_derived.space_screws, share->joints);
    if(!supplied || !described)
        return scene::readout{supplied ? "The described chain has no pose here." : "The supplied chain has no pose here.", {}};

    const evaluation::residual apart = evaluation::pose_residual(supplied.value(), described.value());
    const scene::labeled_value turned{static_cast<float>(apart.magnitude), "Turned from the described chain (rad)"};
    const scene::labeled_value moved{static_cast<float>(apart.linear_error_metres), "Moved from the described chain (m)"};

    return scene::readout{{}, {{turned}, {moved}}};
}

void screw_modeling_window::render_reading()
{
    const scene::readout shown = reading();
    if(!shown.message.empty())
        return ImGui::TextUnformatted(shown.message.c_str());

    for(const std::vector<scene::labeled_value> &line : shown.rows)
        ImGui::Text("%s %.6f", line.front().label.c_str(), static_cast<double>(line.front().value));
}

void screw_modeling_window::render_save()
{
    if(m_save_cb && ImGui::Button("Save chain"))
        m_save_cb(m_settings_at, state());
}

}
