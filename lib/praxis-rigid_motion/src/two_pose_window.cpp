#include "path_sampling.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/two_pose_window.h"

#include <spdlog/spdlog.h>

#include <string>
#include <vector>
#include <utility>

namespace praxis::rigid_motion {

namespace {

const char *const no_motion = "the two poses are equal, so they name no motion";
const char *const no_screw  = "the displacement between the two poses names no screw";

// Exact rather than approximate: what is being asked is whether the pose changed, not whether it
// changed appreciably.
bool same(const transform &first, const transform &second)
{
    return first.cwiseEqual(second).all();
}

Eigen::Vector3d radians_of(const Eigen::Vector3f &degrees)
{
    return Eigen::Vector3d{to_radians(degrees.x()), to_radians(degrees.y()), to_radians(degrees.z())};
}

}

two_pose_window::two_pose_window(std::string name, frame_stencil &target, const capabilities &injected, path_route rebuild)
        : two_pose_window(std::move(name), target, injected, std::move(rebuild),
                          settings{pose_controls{Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), axis_order::xyz},
                                   pose_controls{Eigen::Vector3f::UnitX(), Eigen::Vector3f::Zero(), axis_order::xyz}, 0.f})
{
}

two_pose_window::two_pose_window(std::string name, frame_stencil &target, const capabilities &injected, path_route rebuild, const settings &chosen)
        : imgui_window(std::move(name))
        , m_refused(false)
        , m_reported(false)
        , m_shown(chosen)
        , m_length(0.0)
        , m_magnitude(0.0)
        , m_motions(injected)
        , m_message(no_motion)
        , m_named(std::nullopt)
        , m_applied_start(std::nullopt)
        , m_applied_end(std::nullopt)
        , m_stencil(target)
        , m_rebuild_cb(std::move(rebuild))
{
}

two_pose_window::settings two_pose_window::state() const
{
    return m_shown;
}

transform two_pose_window::start_pose() const
{
    return posed(m_shown.start);
}

transform two_pose_window::end_pose() const
{
    return posed(m_shown.end);
}

scene::readout two_pose_window::reading() const
{
    if(!m_message.empty())
        return scene::readout{m_message, {}};

    return scene::readout{std::string(), {{scene::labeled_value{static_cast<float>(m_length), "Travelled (m)"}}}};
}

void two_pose_window::initialize()
{
    settle();
}

// Keyed on the two poses themselves rather than on a signal standing for them: a pair equal to the
// pair the path was built from is a pair that path is still right for, whichever route the controls
// took back to it.
void two_pose_window::settle()
{
    const transform start = posed(m_shown.start);
    const transform end   = posed(m_shown.end);

    if(!m_applied_start || !m_applied_end || !same(*m_applied_start, start) || !same(*m_applied_end, end))
        return rebuild(start, end);

    place();
}

void two_pose_window::rebuild(const transform &start, const transform &end)
{
    m_applied_start = start;
    m_applied_end   = end;

    name(start, end);

    const std::vector<transform> path = sampled(start);
    m_length                          = path_length(path);
    if(m_rebuild_cb != nullptr)
        m_rebuild_cb(path);

    place();
}

// The screw the two poses name is asked of the bound logarithm and of nothing else. Two poses that
// are equal are refused before it is reached: what it answers for them is a unit slide along x at a
// magnitude of zero, which is a direction the poses never named.
void two_pose_window::name(const transform &start, const transform &end)
{
    m_named.reset();
    m_refused   = false;
    m_magnitude = 0.0;
    if(same(start, end))
        return refuse(no_motion);

    const expected<std::pair<screw_axis, double>, refusal> named = m_motions.screw.matrix_logarithm_se3(end * m_motions.frame.inverse(start));
    if(!named)
    {
        m_refused = true;

        return refuse(no_screw);
    }

    m_reported  = false;
    m_message   = std::string();
    m_named     = named->first;
    m_magnitude = named->second;
}

std::vector<transform> two_pose_window::sampled(const transform &start) const
{
    std::vector<transform> path;
    if(!m_named)
        return path;

    path.reserve(path_points);
    for(std::size_t at = 0; at < path_points; ++at)
        path.push_back(m_motions.screw.matrix_exponential_screw(*m_named, sampled_share(at, path_points) * m_magnitude) * start);

    return path;
}

// A displacement the operations refused is never turned into a pose by guessing one, so the moving
// object is left where it was; two poses that are equal leave it at the start pose, which is where
// every parameter carries it.
void two_pose_window::place()
{
    m_stencil.set_pose(start_object, *m_applied_start);
    m_stencil.set_pose(end_object, *m_applied_end);
    if(m_refused)
        return;

    if(!m_named)
        return m_stencil.set_pose(body_object, *m_applied_start);

    m_stencil.set_pose(body_object, m_motions.screw.matrix_exponential_screw(*m_named, static_cast<double>(m_shown.parameter) * m_magnitude) * *m_applied_start);
}

transform two_pose_window::posed(const pose_controls &shown) const
{
    return m_motions.frame.transformation_matrix_from_rotation_position(m_motions.frame.rotation_matrix_from_euler(radians_of(shown.euler_degrees), shown.order),
                                                                        shown.position.cast<double>());
}

// Reported once per run of refusals rather than once per change: a control that refuses on every
// frame it is asked refuses silently by repetition. The next pair that names a screw clears it.
void two_pose_window::refuse(const char *what)
{
    m_message = what;
    if(std::exchange(m_reported, true))
        return;

    spdlog::warn("praxis: {}, so no path is drawn and the moving object is left where it is", what);
}

}
