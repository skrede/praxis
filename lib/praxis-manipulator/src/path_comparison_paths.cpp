#include "path_comparison_paths.h"

#include "praxis/manipulator/motion_drawings.h"

#include "praxis/rigid_motion/angles.h"

#include <threepp/math/Color.hpp>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <vector>
#include <cstddef>

namespace praxis::manipulator {

namespace {

constexpr threepp::Color::ColorName decoupled_tone = threepp::Color::blue;
constexpr threepp::Color::ColorName screw_tone     = threepp::Color::springgreen;

// The second end a comparison opens at, per joint and in degrees. A joint beyond these opens where
// the first end does, so the pair carries the arm's own width and only its first five values are
// written out.
constexpr std::array<double, 5> opening_offset{120.0, -45.0, 30.0, 90.0, 90.0};

bool unchanged(const Eigen::VectorXf &one, const Eigen::VectorXf &other)
{
    return one.size() == other.size() && one == other;
}

double along(std::size_t step)
{
    return static_cast<double>(step) / static_cast<double>(path_comparison_window::drawn_points - 1u);
}

std::vector<transform> mapped(const forward_kinematics_ops &fk, const screw_chain &chain, const std::vector<joint_vector> &through)
{
    std::vector<transform> reached;
    for(const joint_vector &at : through)
    {
        const expected<transform, refusal> pose = fk.forward_kinematics(chain.home, chain.space_screws, at);
        if(!pose)
            return {};

        reached.push_back(*pose);
    }

    return reached;
}

std::vector<transform> between(expected<transform, refusal> (*shape)(const transform &, const transform &, double), const transform &from, const transform &to)
{
    std::vector<transform> through;
    for(std::size_t step = 0; step < path_comparison_window::drawn_points; ++step)
    {
        const expected<transform, refusal> pose = shape(from, to, along(step));
        if(!pose)
            return {};

        through.push_back(*pose);
    }

    return through;
}

}

compared_path_cycle every_compared_path(compared_path chosen)
{
    return compared_path_cycle{chosen,
                               {compared_path::joint_space, compared_path::decoupled, compared_path::screw},
                               {path_comparison_window::joint_space_path, path_comparison_window::decoupled_path, path_comparison_window::screw_path}};
}

std::vector<joint_vector> configurations_along(const trajectory::path_ops &shapes, const joint_vector &from, const joint_vector &to)
{
    std::vector<joint_vector> through;
    for(std::size_t step = 0; step < path_comparison_window::drawn_points; ++step)
    {
        const expected<trajectory::configuration, refusal> at = shapes.joint_straight_line(from, to, along(step));
        if(!at)
            return {};

        through.push_back(*at);
    }

    return through;
}

joint_vector path_comparison_window::opening_first(std::size_t joints)
{
    return joint_vector::Zero(static_cast<Eigen::Index>(joints));
}

joint_vector path_comparison_window::opening_second(std::size_t joints)
{
    joint_vector taken = joint_vector::Zero(static_cast<Eigen::Index>(joints));
    for(std::size_t joint = 0; joint < joints && joint < opening_offset.size(); ++joint)
        taken[static_cast<Eigen::Index>(joint)] = opening_offset[joint] * radians_per_degree;

    return taken;
}

// The two task-space shapes run between the poses the forward map answers at the two ends, and the
// joint-space one is the forward map at each configuration along the straight line between them.
// No solve enters any of the three.
std::vector<transform> path_comparison_window::poses_along(compared_path shape) const
{
    const joint_vector from = m_first.cast<double>() * radians_per_degree;
    const joint_vector to   = m_second.cast<double>() * radians_per_degree;
    if(shape == compared_path::joint_space)
        return mapped(m_fk, m_chain, configurations_along(m_shapes, from, to));

    const std::vector<transform> ends = mapped(m_fk, m_chain, {from, to});
    if(ends.size() != 2u)
        return {};

    return between(shape == compared_path::screw ? m_shapes.screw : m_shapes.decoupled, ends.front(), ends.back());
}

// Three polylines of sixty-five poses are rebuilt where an end changed and not on every frame.
void path_comparison_window::draw_paths()
{
    if(m_told && unchanged(m_told_first, m_first) && unchanged(m_told_second, m_second))
        return;

    m_told_first  = m_first;
    m_told_second = m_second;
    m_told        = true;

    static_cast<void>(m_drawn.set_pose_path(joint_space_path, poses_along(compared_path::joint_space)));
    static_cast<void>(m_drawn.set_pose_path(decoupled_path, poses_along(compared_path::decoupled), threepp::Color(decoupled_tone)));
    static_cast<void>(m_drawn.set_pose_path(screw_path, poses_along(compared_path::screw), threepp::Color(screw_tone)));
    show_paths();
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
    static_cast<void>(m_drawn.set_pose_path_shown(joint_space_path, m_joint_space));
    static_cast<void>(m_drawn.set_pose_path_shown(decoupled_path, m_decoupled));
    static_cast<void>(m_drawn.set_pose_path_shown(screw_path, m_screw));
}

}
