#include "path_sampling.h"

#include "praxis/rigid_motion/decoupled_path_window.h"

#include <imgui.h>

#include <spdlog/spdlog.h>

#include <string>
#include <vector>
#include <utility>

namespace praxis::rigid_motion {

namespace {

const char *const no_motion = "the two poses are equal, so they name no motion";
const char *const no_turn   = "the rotation between the two poses names no turn";

// Exact rather than approximate: what is being asked is whether the pose changed, not whether it
// changed appreciably.
bool same(const transform &first, const transform &second)
{
    return first.cwiseEqual(second).all();
}

}

decoupled_path_window::decoupled_path_window(std::string name, const capabilities &injected, pose_source between, path_route rebuild)
        : imgui_window(std::move(name))
        , m_reported(false)
        , m_length(0.0)
        , m_turn_radians(0.0)
        , m_motions(injected)
        , m_message(no_motion)
        , m_named(std::nullopt)
        , m_applied_start(std::nullopt)
        , m_applied_end(std::nullopt)
        , m_between(std::move(between))
        , m_rebuild_cb(std::move(rebuild))
{
}

scene::readout decoupled_path_window::reading() const
{
    if(!m_message.empty())
        return scene::readout{m_message, {}};

    return scene::readout{std::string(), {{scene::labeled_value{static_cast<float>(m_length), "Travelled (m)"}}}};
}

void decoupled_path_window::render()
{
    ImGui::Begin(display_name().c_str());
    ImGui::TextUnformatted("The orientation and the position carried independently");
    if(!m_message.empty())
        ImGui::TextUnformatted(m_message.c_str());
    else
        ImGui::Text("Travelled %.3f m", m_length);
    ImGui::End();

    settle();
}

void decoupled_path_window::initialize()
{
    settle();
}

// Keyed on the two poses themselves rather than on a signal standing for them, and read from the
// scenario every frame rather than held: a pair equal to the pair the path was built from is a pair
// that path is still right for.
void decoupled_path_window::settle()
{
    if(m_between == nullptr)
        return;

    const std::pair<transform, transform> between = m_between();
    if(m_applied_start && m_applied_end && same(*m_applied_start, between.first) && same(*m_applied_end, between.second))
        return;

    rebuild(between.first, between.second);
}

void decoupled_path_window::rebuild(const transform &start, const transform &end)
{
    m_applied_start = start;
    m_applied_end   = end;

    name(start, end);

    const std::vector<transform> path = sampled(start, end);
    m_length                          = path_length(path);
    if(m_rebuild_cb != nullptr)
        m_rebuild_cb(path);
}

// The turn between the two orientations is asked of the bound rotation logarithm and of nothing
// else. Two poses that are equal are refused before it is reached: what it answers for them is a
// unit x-direction at an angle of zero, which is an axis the poses never named.
void decoupled_path_window::name(const transform &start, const transform &end)
{
    m_named.reset();
    m_turn_radians = 0.0;
    if(same(start, end))
        return refuse(no_motion);

    const rotation from = m_motions.frame.rotation_matrix_from_transform(m_motions.frame.inverse(start));
    const rotation to   = m_motions.frame.rotation_matrix_from_transform(end);

    const expected<std::pair<Eigen::Vector3d, double>, refusal> turned = m_motions.screw.matrix_logarithm_so3(to * from);
    if(!turned)
        return refuse(no_turn);

    m_reported     = false;
    m_message      = std::string();
    m_named        = turned->first;
    m_turn_radians = turned->second;
}

// The orientation is carried by the bound rotation exponential of the parameter's share of the turn
// the bound rotation logarithm answered; the position is carried along the straight line between
// the two, which is the only part of this path the scenario does itself.
std::vector<transform> decoupled_path_window::sampled(const transform &start, const transform &end) const
{
    std::vector<transform> path;
    if(!m_named)
        return path;

    const rotation from      = m_motions.frame.rotation_matrix_from_transform(start);
    const Eigen::Vector3d at = start.block<3, 1>(0, 3);
    const Eigen::Vector3d to = end.block<3, 1>(0, 3);

    path.reserve(two_pose_window::path_points);
    for(std::size_t step = 0; step < two_pose_window::path_points; ++step)
    {
        const double along = sampled_share(step, two_pose_window::path_points);
        path.push_back(m_motions.frame.transformation_matrix_from_rotation_position(m_motions.screw.matrix_exponential_so3(*m_named, along * m_turn_radians) * from,
                                                                                    (1.0 - along) * at + along * to));
    }

    return path;
}

// Reported once per run of refusals rather than once per change: a control that refuses on every
// frame it is asked refuses silently by repetition. The next pair that names a turn clears it.
void decoupled_path_window::refuse(const char *what)
{
    m_message = what;
    if(std::exchange(m_reported, true))
        return;

    spdlog::warn("praxis: {}, so no decoupled path is drawn", what);
}

}
