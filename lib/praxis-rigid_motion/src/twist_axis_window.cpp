#include "praxis/rigid_motion/screw_widgets.h"
#include "praxis/rigid_motion/twist_axis_window.h"

#include <spdlog/spdlog.h>

#include <string>
#include <vector>
#include <utility>

namespace praxis::rigid_motion {

namespace {

// Exact rather than approximate: what is being asked is whether the value changed, not whether it
// changed appreciably.
bool same(const Eigen::Vector3f &first, const Eigen::Vector3f &second)
{
    return first.cwiseEqual(second).all();
}

}

twist_axis_window::twist_axis_window(std::string name, frame_stencil &target, const capabilities &injected, axis_route rebuild)
        : twist_axis_window(std::move(name), target, injected, std::move(rebuild), settings{Eigen::Vector3f::UnitZ(), Eigen::Vector3f::Zero(), 0.f})
{
}

twist_axis_window::twist_axis_window(std::string name, frame_stencil &target, const capabilities &injected, axis_route rebuild, const settings &chosen)
        : imgui_window(std::move(name))
        , m_reported(false)
        , m_shown(chosen)
        , m_start(target.pose(body_object))
        , m_motions(injected)
        , m_applied(std::nullopt)
        , m_named(std::nullopt)
        , m_stencil(target)
        , m_rebuild_cb(std::move(rebuild))
{
}

twist_axis_window::settings twist_axis_window::state() const
{
    return m_shown;
}

scene::readout twist_axis_window::reading() const
{
    if(!m_named)
        return scene::readout{"the twist names no axis", {}};

    std::vector<scene::labeled_value> cells;
    cells.reserve(static_cast<std::size_t>(m_named->size()));
    for(Eigen::Index at = 0; at < m_named->size(); ++at)
        cells.push_back(scene::labeled_value{static_cast<float>((*m_named)[at]), std::string()});

    return scene::readout{std::string(), {std::move(cells)}};
}

void twist_axis_window::render()
{
    const auto limit = static_cast<float>(screw_window::angle_limit_radians);

    ImGui::Begin(display_name().c_str());
    static_cast<void>(render_twist_inputs(m_shown.angular_part, m_shown.linear_part));
    ImGui::SliderFloat("Angle", &m_shown.angle_radians, -limit, limit);
    ImGui::End();

    settle();
}

void twist_axis_window::initialize()
{
    rebuild();
}

// Keyed on the twist itself rather than on a signal standing for it: a twist equal to the one the
// axis was last built from is one that axis is still right for, whichever route the values took
// back to it.
void twist_axis_window::settle()
{
    if(!m_applied || !same(m_applied->angular_part, m_shown.angular_part) || !same(m_applied->linear_part, m_shown.linear_part))
        return rebuild();

    if(m_applied->angle_radians != m_shown.angle_radians)
        apply();
}

void twist_axis_window::rebuild()
{
    m_named.reset();
    if(m_shown.angular_part.isZero() && m_shown.linear_part.isZero())
        refuse("the twist's angular part and its linear part are both zero and name no axis");
    else
    {
        m_reported = false;
        m_named    = m_motions.screw.screw_axis_from_angular_linear(m_shown.angular_part.cast<double>(), m_shown.linear_part.cast<double>());
    }

    if(m_rebuild_cb != nullptr)
        m_rebuild_cb(m_named);

    apply();
}

void twist_axis_window::apply()
{
    m_applied = m_shown;
    if(!m_named)
        return;

    m_stencil.set_pose(body_object, m_motions.screw.matrix_exponential_screw(*m_named, m_shown.angle_radians) * m_start);
}

// Reported once per run of refusals rather than once per change: a control that refuses on every
// frame it is asked refuses silently by repetition. The next twist that names an axis clears it.
void twist_axis_window::refuse(const char *what)
{
    if(std::exchange(m_reported, true))
        return;

    spdlog::warn("praxis: {}, so nothing is drawn and the scene is left where it was", what);
}

}
