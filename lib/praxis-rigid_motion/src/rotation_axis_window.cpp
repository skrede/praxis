#include "praxis/rigid_motion/screw_widgets.h"
#include "praxis/rigid_motion/rotation_axis_window.h"

#include <spdlog/spdlog.h>

#include <string>
#include <utility>

namespace praxis::rigid_motion {

namespace {

// Exact rather than approximate: what is being asked is whether the value changed, not whether it
// changed appreciably.
bool same(const rotation_axis_window::settings &first, const rotation_axis_window::settings &second)
{
    return first.direction.cwiseEqual(second.direction).all() && first.angle_radians == second.angle_radians && first.axis_shown == second.axis_shown &&
            first.coordinate_shown == second.coordinate_shown && first.frame_shown == second.frame_shown && first.arc_shown == second.arc_shown;
}

}

rotation_axis_window::rotation_axis_window(std::string name, frame_stencil &target, const capabilities &injected, axis_route rebuild)
        : rotation_axis_window(std::move(name), target, injected, std::move(rebuild), settings{Eigen::Vector3f::UnitZ(), 0.f, true, true, true, true})
{
}

rotation_axis_window::rotation_axis_window(std::string name, frame_stencil &target, const capabilities &injected, axis_route rebuild, const settings &chosen)
        : imgui_window(std::move(name))
        , m_reported(false)
        , m_shown(chosen)
        , m_start(target.pose(frame_object))
        , m_motions(injected)
        , m_applied(std::nullopt)
        , m_named(std::nullopt)
        , m_stencil(target)
        , m_rebuild_cb(std::move(rebuild))
{
}

rotation_axis_window::settings rotation_axis_window::state() const
{
    return m_shown;
}

void rotation_axis_window::render()
{
    const auto limit = static_cast<float>(angle_limit_radians);

    ImGui::Begin(display_name().c_str());
    static_cast<void>(render_direction_input(m_shown.direction));
    ImGui::SliderFloat("Angle", &m_shown.angle_radians, -limit, limit);
    ImGui::SameLine();
    if(ImGui::Button("Reset"))
        m_shown.angle_radians = 0.f;

    ImGui::Checkbox("Unit axis", &m_shown.axis_shown);
    ImGui::Checkbox("Coordinate vector", &m_shown.coordinate_shown);
    ImGui::Checkbox("Traversed arc", &m_shown.arc_shown);
    ImGui::Checkbox("Frame", &m_shown.frame_shown);
    ImGui::End();

    settle();
}

void rotation_axis_window::initialize()
{
    settle();
}

void rotation_axis_window::settle()
{
    if(m_applied && same(*m_applied, m_shown))
        return;

    m_named.reset();
    if(m_shown.direction.isZero())
        refuse("the axis direction is zero and names no axis");
    else
    {
        m_reported = false;
        m_named    = m_shown.direction.cast<double>().normalized();
    }

    apply();

    if(m_rebuild_cb != nullptr)
        m_rebuild_cb(m_shown, m_named);
}

void rotation_axis_window::apply()
{
    m_applied = m_shown;
    if(!m_named)
        return;

    m_stencil.set_pose(frame_object, m_motions.frame.transformation_matrix_from_rotation(m_motions.screw.matrix_exponential_so3(*m_named, m_shown.angle_radians)) * m_start);
}

// Reported once per run of refusals rather than once per change: a control that refuses on every
// frame it is asked refuses silently by repetition. The next direction naming an axis clears it.
void rotation_axis_window::refuse(const char *what)
{
    if(std::exchange(m_reported, true))
        return;

    spdlog::warn("praxis: {}, so nothing is applied and the scene is left where it was", what);
}

}
