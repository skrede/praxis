#include "praxis/rigid_motion/screw_widgets.h"
#include "praxis/rigid_motion/screw_window.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <string>
#include <utility>
#include <algorithm>

namespace praxis::rigid_motion {

namespace {

// The world z-axis is carried onto the direction by turning about their cross product through the
// angle between them. That cross product vanishes when the two are parallel or antiparallel, and
// the world x-axis stands in for it there: the parallel case turns by nothing whatever axis it is
// given, and the antiparallel case is a half turn about any axis perpendicular to z.
Eigen::Vector3d turn_axis(const Eigen::Vector3d &along)
{
    const Eigen::Vector3d across = Eigen::Vector3d::UnitZ().cross(along);

    return across.isZero() ? Eigen::Vector3d::UnitX() : Eigen::Vector3d(across.normalized());
}

double turn_angle(const Eigen::Vector3d &along)
{
    return std::acos(std::clamp(along.z(), -1.0, 1.0));
}

}

screw_window::screw_window(std::string name, frame_stencil &target, const capabilities &injected, axis_route rebuild)
        : screw_window(std::move(name), target, injected, std::move(rebuild), settings{Eigen::Vector3f::Zero(), Eigen::Vector3f::UnitZ(), 0.f, 0.f})
{
}

screw_window::screw_window(std::string name, frame_stencil &target, const capabilities &injected, axis_route rebuild, const settings &chosen)
        : imgui_window(std::move(name))
        , m_pitch(chosen.pitch)
        , m_angle_radians(chosen.angle_radians)
        , m_reported(false)
        , m_start(target.pose(body_object))
        , m_motions(injected)
        , m_point(chosen.point)
        , m_direction(chosen.direction)
        , m_named(std::nullopt)
        , m_stencil(target)
        , m_rebuild_cb(std::move(rebuild))
{
}

screw_window::settings screw_window::state() const
{
    return settings{m_point, m_direction, m_pitch, m_angle_radians};
}

void screw_window::render()
{
    ImGui::Begin(display_name().c_str());
    if(render_screw_inputs(m_point, m_direction, m_pitch))
        rebuild();
    if(ImGui::SliderFloat("Angle", &m_angle_radians, -static_cast<float>(angle_limit_radians), static_cast<float>(angle_limit_radians)))
        apply();
    ImGui::End();
}

void screw_window::initialize()
{
    rebuild();
}

void screw_window::apply()
{
    if(!m_named)
        return;

    place_axis();
    m_stencil.set_pose(body_object, m_motions.screw.matrix_exponential_screw(*m_named, m_angle_radians) * m_start);
}

// The axis is derived once here and held, so everything drawn from it and every pose driven by it
// answer to the one axis the controls named rather than to a second derivation of it.
void screw_window::rebuild()
{
    m_named.reset();
    if(m_direction.isZero())
        refuse("the axis direction is zero and names no axis");
    else if(const expected<screw_axis, refusal> about = m_motions.screw.screw_axis_from_point_direction_pitch(m_point.cast<double>(), m_direction.cast<double>(), m_pitch); !about)
        refuse("screw.screw_axis_from_point_direction_pitch named no axis");
    else
    {
        m_reported = false;
        m_named    = *about;
    }

    if(m_rebuild_cb != nullptr)
        m_rebuild_cb(state(), m_named);

    apply();
}

void screw_window::place_axis()
{
    const Eigen::Vector3d along = m_direction.cast<double>().normalized();
    const rotation onto         = m_motions.screw.matrix_exponential_so3(turn_axis(along), turn_angle(along));

    m_stencil.set_pose(axis_object, m_motions.frame.transformation_matrix_from_rotation_position(onto, m_point.cast<double>()));
}

// Reported once per run of refusals rather than once per change: a control that refuses on every
// frame it is asked refuses silently by repetition. The next axis the controls name clears it.
void screw_window::refuse(const char *what)
{
    if(std::exchange(m_reported, true))
        return;

    spdlog::warn("praxis: {}, so nothing is applied and the scene is left where it was", what);
}

}
