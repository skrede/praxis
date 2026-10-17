#include "praxis/manipulator/motion_drawings.h"
#include "praxis/manipulator/path_comparison_window.h"

#include "robot/pose_path.h"

#include "praxis/extension/refusal.h"

#include <spdlog/spdlog.h>

#include <threepp/math/Color.hpp>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <vector>
#include <cstddef>
#include <string_view>

namespace praxis::manipulator {

namespace {

constexpr threepp::Color::ColorName decoupled_tone = threepp::Color::blue;
constexpr threepp::Color::ColorName screw_tone     = threepp::Color::springgreen;

// In `compared_path` order.
constexpr std::array<const char *, 3> drawn_names{path_comparison_window::joint_space_path, path_comparison_window::decoupled_path, path_comparison_window::screw_path};

bool unchanged(const Eigen::VectorXf &one, const Eigen::VectorXf &other)
{
    return one.size() == other.size() && one == other;
}

// The forward map answers flange poses and all four polylines are drawn at the tool centre point.
std::vector<transform> at_the_tool(const robot_ops &robot, const transform &tool_offset, const std::vector<transform> &flange_poses)
{
    std::vector<transform> standing;
    standing.reserve(flange_poses.size());
    for(const transform &pose : flange_poses)
        standing.push_back(robot.tool_pose_from_flange_pose(pose, tool_offset));

    return standing;
}

threepp::Color drawn_tone(compared_path shape)
{
    if(shape == compared_path::joint_space)
        return opening_path_tone();

    return threepp::Color(shape == compared_path::screw ? screw_tone : decoupled_tone);
}

const char *refusal_name(refusal reason)
{
    switch(reason)
    {
        case refusal::unsupported_input:
            return "unsupported input";
        case refusal::degenerate:
            return "degenerate";
        case refusal::no_solution:
            return "no solution";
        case refusal::not_implemented:
            return "not implemented";
    }

    return "unclassified";
}

void report_refused(std::string_view slot, refusal reason, const char *named)
{
    if(reason == refusal::not_implemented)
        spdlog::warn("praxis: '{}' is not implemented, so the path told under \"{}\" is not drawn", slot, named);
    else
        spdlog::error("praxis: '{}' refused the path told under \"{}\" as {}, so it is not drawn", slot, named, refusal_name(reason));
}

}

// Three polylines of sixty-five poses are rebuilt where an end or the tool offset they are carried to
// moved, and not on every frame.
void path_comparison_window::draw_paths()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();
    if(published == nullptr)
        return;

    if(m_told && unchanged(m_told_first, m_first) && unchanged(m_told_second, m_second) && m_told_offset == published->tool_offset)
        return;

    m_told_first  = m_first;
    m_told_second = m_second;
    m_told_offset = published->tool_offset;
    m_told        = true;

    for(const compared_path shape : {compared_path::joint_space, compared_path::decoupled, compared_path::screw})
        draw_path(shape);
    show_paths();
}

void path_comparison_window::draw_path(compared_path shape)
{
    const std::size_t at                                     = static_cast<std::size_t>(shape);
    const expected<std::vector<transform>, refused_slot> run = sampled(shape);
    if(run)
    {
        m_refused[at].reset();
        static_cast<void>(m_drawn.set_pose_path(drawn_names[at], at_the_tool(m_robot, m_told_offset, *run), drawn_tone(shape)));

        return;
    }

    m_drawn.clear_pose_path(drawn_names[at]);
    if(m_refused[at] != run.error().reason)
        report_refused(run.error().slot, run.error().reason, drawn_names[at]);

    m_refused[at] = run.error().reason;
}

// The run the tool traversed grows while a motion plays, so it is told on its own occasion rather
// than behind the guard that holds the three commanded shapes still while neither end has moved.
void path_comparison_window::draw_traversed()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();
    if(published == nullptr || published->traversed.get() == m_told_traversed.get())
        return;

    m_told_traversed = published->traversed;
    if(m_told_traversed != nullptr && m_told_traversed->size() >= least_drawn_poses)
        static_cast<void>(m_drawn.set_pose_path(traversed_motion_path, *m_told_traversed, threepp::Color(traversed_motion_tone)));
}

void path_comparison_window::show_paths()
{
    const std::array<bool, 3> shown{m_joint_space, m_decoupled, m_screw};
    for(std::size_t at = 0; at < drawn_names.size(); ++at)
    {
        if(!m_refused[at])
            static_cast<void>(m_drawn.set_pose_path_shown(drawn_names[at], shown[at]));
    }
}

}
