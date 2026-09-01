#include "robot/pose_path.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <spdlog/spdlog.h>

#include <threepp/math/Color.hpp>

#include <Eigen/Core>

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <string_view>

namespace praxis::manipulator {

namespace {

std::vector<Eigen::Vector3d> path_points(std::span<const transform> through)
{
    std::vector<Eigen::Vector3d> points;
    points.reserve(through.size());
    for(const transform &at : through)
        points.push_back(at.block<3, 1>(0, 3));

    return points;
}

}

std::string loadable_robot_stencil::pose_path_name(std::string_view named)
{
    return "Tool path " + std::string(named);
}

expected<void, refusal> loadable_robot_stencil::set_pose_path(std::string_view named, std::span<const transform> through)
{
    return set_pose_path(named, through, opening_path_tone());
}

// The paths group hangs under the decoration root, which already carries the quarter turn matching
// the rendered robot's, so a position taken straight out of a space-frame pose needs no conversion.
expected<void, refusal> loadable_robot_stencil::set_pose_path(std::string_view named, std::span<const transform> through, threepp::Color tone)
{
    const std::string under = pose_path_name(named);

    std::shared_ptr<threepp::Object3D> drawn = pose_path_object(under, path_points(through), tone);
    if(drawn == nullptr)
    {
        spdlog::error("praxis: a drawn path joins two tool poses at least and the one told under \"{}\" names {}, so it is not drawn", named, through.size());

        return unexpected(refusal::unsupported_input);
    }

    // Read before the clear, which erases the entry the switch is read from.
    const bool as_before = path_shown_or_opening(under);

    clear_pose_path(named);
    drawn->visible = as_before;
    m_path_lines.push_back(drawn_path{under, std::move(drawn)});
    m_paths->add(m_path_lines.back().object);

    return {};
}

void loadable_robot_stencil::clear_pose_path(std::string_view named)
{
    const auto standing = standing_path(pose_path_name(named));
    if(standing == m_path_lines.end())
        return;

    m_paths->remove(*standing->object);
    m_path_lines.erase(standing);
}

void loadable_robot_stencil::clear_pose_paths()
{
    for(const drawn_path &at : m_path_lines)
        m_paths->remove(*at.object);

    m_path_lines.clear();
}

expected<void, refusal> loadable_robot_stencil::set_pose_path_shown(std::string_view named, bool shown)
{
    const auto standing = standing_path(pose_path_name(named));
    if(standing == m_path_lines.end())
    {
        spdlog::error("praxis: the drawing carries no path told under \"{}\", so showing or hiding it is declined", named);

        return unexpected(refusal::unsupported_input);
    }

    standing->object->visible = shown;

    return {};
}

std::vector<loadable_robot_stencil::drawn_path>::iterator loadable_robot_stencil::standing_path(const std::string &under)
{
    return std::find_if(m_path_lines.begin(), m_path_lines.end(), [&under](const drawn_path &at) { return at.name == under; });
}

bool loadable_robot_stencil::path_shown_or_opening(const std::string &under)
{
    const auto standing = standing_path(under);

    return standing == m_path_lines.end() || standing->object->visible;
}

}
