#include "velocity_kinematics_rows.h"

#include "robot/column_arrow.h"

#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/velocity_kinematics_window.h"

#include <imgui.h>

#include <threepp/math/Color.hpp>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <cstddef>

namespace praxis::manipulator {

namespace {

constexpr std::size_t angular = static_cast<std::size_t>(jacobian_block::angular);
constexpr std::size_t linear  = static_cast<std::size_t>(jacobian_block::linear);

// A part's tone in the colour space the renderer encodes to on output, which is the space this panel
// writes its own colours in.
ImU32 as_written(jacobian_block part)
{
    const unsigned int worn = column_tone(part).getHex(threepp::SRGBColorSpace);

    return IM_COL32((worn >> 16) & 0xffu, (worn >> 8) & 0xffu, worn & 0xffu, 0xff);
}

void name_part(const char *called, jacobian_block part)
{
    const float side = ImGui::GetTextLineHeight();
    const ImVec2 at  = ImGui::GetCursorScreenPos();

    ImGui::GetWindowDrawList()->AddRectFilled(at, ImVec2(at.x + side, at.y + side), as_written(part));
    ImGui::Dummy(ImVec2(side, side));
    ImGui::SameLine();
    ImGui::TextUnformatted(called);
}

void name_column_parts()
{
    ImGui::Indent();
    name_part("Angular part", jacobian_block::angular);
    name_part("Linear part", jacobian_block::linear);
    ImGui::Unindent();
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
        , m_shown{state.angular_ellipsoid, state.linear_ellipsoid}
        , m_refused(false)
        , m_controls(offered)
        , m_settings_at(std::move(at))
        , m_seen(std::move(seen))
        , m_arm(std::move(arm))
        , m_drawn(drawn)
        , m_frame(state.frame, {jacobian_frame::space, jacobian_frame::body}, {"Space", "Body"})
        , m_reading(state.reading, {ellipsoid_view::velocity, ellipsoid_view::force}, {"Velocity", "Force"})
{
}

velocity_kinematics_window::settings velocity_kinematics_window::state() const
{
    return settings{.frame             = m_frame.value(),
                    .reading           = m_reading.value(),
                    .angular_ellipsoid = m_shown[angular],
                    .linear_ellipsoid  = m_shown[linear],
                    .columns           = m_columns,
                    .capped            = m_capped};
}

void velocity_kinematics_window::initialize()
{
    m_drawn.set_jacobian_frame(m_frame.value());
    m_drawn.set_manipulability_ellipsoids(m_reading.value());
    m_drawn.set_angular_ellipsoid_shown(m_shown[angular]);
    m_drawn.set_linear_ellipsoid_shown(m_shown[linear]);
    m_drawn.set_jacobian_columns_shown(m_columns);
    m_drawn.set_force_capped(m_capped);
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

void velocity_kinematics_window::render_switches()
{
    if(ImGui::Checkbox("Angular ellipsoid", &m_shown[angular]))
        m_drawn.set_angular_ellipsoid_shown(m_shown[angular]);
    if(ImGui::Checkbox("Linear ellipsoid", &m_shown[linear]))
        m_drawn.set_linear_ellipsoid_shown(m_shown[linear]);
    if(ImGui::Checkbox("Jacobian columns", &m_columns))
        m_drawn.set_jacobian_columns_shown(m_columns);
    if(m_columns)
        name_column_parts();
    if(ImGui::Checkbox("Cap the force ellipsoid", &m_capped))
        m_drawn.set_force_capped(m_capped);
}

scene::readout velocity_kinematics_window::reading() const
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();
    const ellipsoid_view read                           = m_reading.value();
    const double lengths[jacobian_block_count]{m_drawn.ellipsoid_scale(jacobian_block::angular), m_drawn.ellipsoid_scale(jacobian_block::linear)};
    scene::readout answered = velocity_kinematics_reading(published.get(), m_frame.value(), read, lengths[angular], lengths[linear]);

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
