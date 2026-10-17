#include "path_comparison_paths.h"

#include "praxis/manipulator/slots.h"
#include "praxis/manipulator/motion_commands.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/angles.h"

#include "praxis/trajectory/slots.h"

#include "praxis/extension/coverage.h"

#include <Eigen/Core>

#include <array>
#include <vector>
#include <cstddef>
#include <utility>
#include <string_view>

namespace praxis::manipulator {

namespace {

// The second end a comparison opens at, per joint and in degrees. A joint beyond these opens where
// the first end does, so the pair carries the arm's own width and only its first five values are
// written out.
constexpr std::array<double, 5> opening_offset{120.0, -45.0, 30.0, 90.0, 90.0};

double along(std::size_t step)
{
    return static_cast<double>(step) / static_cast<double>(path_comparison_window::drawn_points - 1u);
}

expected<std::vector<transform>, refusal> mapped(const rigid_motion::screw_ops &screw, const forward_kinematics_ops &fk, const screw_chain &chain,
                                                 const std::vector<joint_vector> &through)
{
    std::vector<transform> reached;
    for(const joint_vector &at : through)
    {
        const expected<transform, refusal> pose = fk.forward_kinematics(screw, chain.home, chain.space_screws, at);
        if(!pose)
            return praxis::unexpected(pose.error());

        reached.push_back(*pose);
    }

    return reached;
}

expected<std::vector<transform>, refusal> between(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, task_space_path shape, const transform &from,
                                                  const transform &to)
{
    std::vector<transform> through;
    for(std::size_t step = 0; step < path_comparison_window::drawn_points; ++step)
    {
        const expected<transform, refusal> pose = shape(screw, frames, from, to, along(step));
        if(!pose)
            return praxis::unexpected(pose.error());

        through.push_back(*pose);
    }

    return through;
}

template<typename Ops>
std::string_view slot_called(const Ops &described, capability_slots_t<Ops> which)
{
    return slot_name(view_of(described), static_cast<std::size_t>(which));
}

}

compared_path_cycle every_compared_path(compared_path chosen)
{
    return compared_path_cycle{chosen,
                               {compared_path::joint_space, compared_path::decoupled, compared_path::screw},
                               {path_comparison_window::joint_space_path, path_comparison_window::decoupled_path, path_comparison_window::screw_path}};
}

expected<std::vector<joint_vector>, refusal> configurations_along(const trajectory::path_ops &shapes, const joint_vector &from, const joint_vector &to)
{
    std::vector<joint_vector> through;
    for(std::size_t step = 0; step < path_comparison_window::drawn_points; ++step)
    {
        const expected<trajectory::configuration, refusal> at = shapes.joint_straight_line(from, to, along(step));
        if(!at)
            return praxis::unexpected(at.error());

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
expected<std::vector<transform>, path_comparison_window::refused_slot> path_comparison_window::sampled(compared_path shape) const
{
    const joint_vector from = m_first.cast<double>() * radians_per_degree;
    const joint_vector to   = m_second.cast<double>() * radians_per_degree;
    const bool joint_space  = shape == compared_path::joint_space;

    const expected<std::vector<joint_vector>, refusal> through = joint_space ? configurations_along(m_shapes, from, to) : std::vector<joint_vector>{from, to};
    if(!through)
        return praxis::unexpected(refused_slot{slot_called(m_shapes, trajectory::path_slot::joint_straight_line), through.error()});

    expected<std::vector<transform>, refusal> reached = mapped(m_screw_ops, m_fk, m_chain, *through);
    if(!reached)
        return praxis::unexpected(refused_slot{slot_called(m_fk, forward_kinematics_slot::forward_kinematics), reached.error()});

    if(joint_space)
        return std::move(*reached);

    const trajectory::path_slot task_space = shape == compared_path::screw ? trajectory::path_slot::screw : trajectory::path_slot::decoupled;
    expected<std::vector<transform>, refusal> run =
            between(m_screw_ops, m_frame_ops, shape == compared_path::screw ? m_shapes.screw : m_shapes.decoupled, reached->front(), reached->back());
    if(!run)
        return praxis::unexpected(refused_slot{slot_called(m_shapes, task_space), run.error()});

    return std::move(*run);
}

std::vector<transform> path_comparison_window::poses_along(compared_path shape) const
{
    expected<std::vector<transform>, refused_slot> run = sampled(shape);

    return run ? std::move(*run) : std::vector<transform>();
}

}
