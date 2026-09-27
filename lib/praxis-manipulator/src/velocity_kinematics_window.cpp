#include "joint_naming.h"
#include "velocity_kinematics_rows.h"

#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/velocity_kinematics_window.h"

#include "praxis/scene/widgets.h"

#include <imgui.h>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <cstddef>
#include <optional>

namespace praxis::manipulator {

namespace {

constexpr std::size_t angular = static_cast<std::size_t>(jacobian_block::angular);
constexpr std::size_t linear  = static_cast<std::size_t>(jacobian_block::linear);

// The switch over one part, standing beside the tone that part's arrows wear: the two tones are
// neighbours by construction, so which part is which is not answerable from the drawing alone.
bool switch_over(const char *called, jacobian_block part, bool &shown)
{
    const float side = ImGui::GetTextLineHeight();
    const ImVec2 at  = ImGui::GetCursorScreenPos();

    ImGui::GetWindowDrawList()->AddRectFilled(at, ImVec2(at.x + side, at.y + side), as_written(part));
    ImGui::Dummy(ImVec2(side, side));
    ImGui::SameLine();

    return ImGui::Checkbox(called, &shown);
}

std::vector<std::string> highlight_entries(const loadable_robot_stencil &drawn)
{
    std::vector<std::string> entries{"None"};
    for(std::string &named : named_joints(drawn.robot().numDOF()))
        entries.push_back(std::move(named));

    return entries;
}

}

velocity_kinematics_window::velocity_kinematics_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, loadable_robot_stencil &drawn)
        : velocity_kinematics_window(std::move(name), std::move(seen), std::move(arm), drawn, controls(), settings{})
{
}

velocity_kinematics_window::velocity_kinematics_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, loadable_robot_stencil &drawn, const controls &offered,
                                                       const settings &state, std::string at)
        : labeled_value_window(
                  std::move(name), [this] { render_controls(); }, [this] { return reading(); })
        , m_capped(state.capped)
        , m_columns(state.columns)
        , m_ellipsoids(state.ellipsoids)
        , m_shown{state.angular, state.linear}
        , m_refused(false)
        , m_controls(offered)
        , m_settings_at(std::move(at))
        , m_seen(std::move(seen))
        , m_arm(std::move(arm))
        , m_drawn(drawn)
        , m_frame(state.frame, {jacobian_frame::space, jacobian_frame::body}, {"Space", "Body"})
        , m_highlighted(state.highlighted)
        , m_reading(state.reading, {ellipsoid_view::velocity, ellipsoid_view::force}, {"Velocity", "Force"})
        , m_highlight_entries(highlight_entries(drawn))
{
}

velocity_kinematics_window::settings velocity_kinematics_window::state() const
{
    return settings{.frame       = m_frame.value(),
                    .reading     = m_reading.value(),
                    .angular     = m_shown[angular],
                    .linear      = m_shown[linear],
                    .columns     = m_columns,
                    .capped      = m_capped,
                    .ellipsoids  = m_ellipsoids,
                    .highlighted = m_highlighted};
}

void velocity_kinematics_window::initialize()
{
    m_drawn.set_jacobian_frame(m_frame.value());
    m_drawn.set_manipulability_ellipsoids(m_reading.value());
    apply_part(jacobian_block::angular);
    apply_part(jacobian_block::linear);
    m_drawn.set_jacobian_columns_shown(m_columns);
    m_drawn.set_force_capped(m_capped);
    tell_highlighted();
}

// The drawing names a joint it declines, and the window then stands at none with it.
void velocity_kinematics_window::tell_highlighted()
{
    if(m_highlighted && m_drawn.set_selected_joint(*m_highlighted))
        return;

    m_highlighted.reset();
    m_drawn.clear_selected_joint();
}

void velocity_kinematics_window::apply_part(jacobian_block which) const
{
    const bool shown = m_shown[static_cast<std::size_t>(which)];

    if(which == jacobian_block::angular)
        m_drawn.set_angular_ellipsoid_shown(m_ellipsoids && shown);
    else
        m_drawn.set_linear_ellipsoid_shown(m_ellipsoids && shown);

    m_drawn.set_column_part_shown(which, shown);
}

void velocity_kinematics_window::render_controls()
{
    if(m_controls.frame)
        render_frame();
    if(m_controls.reading)
        render_reading();
    if(m_controls.shown)
        render_switches();
}

// One control, three drawings: the matrix read below, both ellipsoids and the drawn columns are all
// taken from whichever Jacobian this names.
void velocity_kinematics_window::render_frame()
{
    if(render_option_cycle("Jacobian", m_frame))
        m_drawn.set_jacobian_frame(m_frame.value());
}

void velocity_kinematics_window::render_reading()
{
    if(render_option_cycle("Ellipsoid", m_reading))
        m_drawn.set_manipulability_ellipsoids(m_reading.value());
}

// A part's switch reaches both of that part's drawings, and the ellipsoids and columns switches reach
// those drawings at all: either turned off leaves the other kind where the part switches put it, and
// the ellipsoids switch turned on puts each ellipsoid back where its part's switch says.
void velocity_kinematics_window::render_switches()
{
    if(switch_over("Angular", jacobian_block::angular, m_shown[angular]))
        apply_part(jacobian_block::angular);
    if(switch_over("Linear", jacobian_block::linear, m_shown[linear]))
        apply_part(jacobian_block::linear);
    if(ImGui::Checkbox("Ellipsoids", &m_ellipsoids))
    {
        apply_part(jacobian_block::angular);
        apply_part(jacobian_block::linear);
    }
    if(ImGui::Checkbox("Jacobian columns", &m_columns))
        m_drawn.set_jacobian_columns_shown(m_columns);
    render_highlighted();
    if(ImGui::Checkbox("Cap the force ellipsoid", &m_capped))
        m_drawn.set_force_capped(m_capped);
}

void velocity_kinematics_window::render_highlighted()
{
    std::size_t entry = m_highlighted ? *m_highlighted + 1u : 0u;
    if(!scene::render_dropdown_selection("Show joint", entry, m_highlight_entries))
        return;

    m_highlighted = entry == 0u ? std::nullopt : std::optional<std::size_t>(entry - 1u);
    tell_highlighted();
}

scene::readout velocity_kinematics_window::reading() const
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();
    const ellipsoid_view read                           = m_reading.value();
    const double lengths[jacobian_block_count]{m_drawn.ellipsoid_scale(jacobian_block::angular), m_drawn.ellipsoid_scale(jacobian_block::linear)};
    scene::readout answered = velocity_kinematics_reading(published.get(), m_frame.value(), read, lengths[angular], lengths[linear], m_drawn.selected_joint());

    const bool runaway = published != nullptr &&
            either_ellipsoid_unbounded(m_frame == jacobian_frame::space ? published->space_manipulability : published->body_manipulability, read, lengths[angular], lengths[linear]);
    if(runaway && !std::exchange(m_refused, true))
        report_unbounded();
    if(!runaway)
        m_refused = false;

    return answered;
}

void velocity_kinematics_window::report_unbounded() const
{
    command(m_arm, [](robot_controller &control, scene_robot &) { control.report_refusal(unbounded_ellipsoid, refusal::no_solution); });
}

}
