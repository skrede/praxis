#include "praxis/manipulator/motion_commands.h"
#include "praxis/manipulator/baseline/task_trajectory.h"

#include "joint_turns.h"

#include "praxis/rigid_motion/angles.h"

#include <spdlog/spdlog.h>

#include <Eigen/Core>

#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>

namespace praxis::manipulator {

namespace {

// Degrees, the largest turn of one joint between consecutive configurations before it is reported.
constexpr double reported_joint_turn_degrees = 30.0;

struct joint_turn
{
    double degrees     = 0.0;
    Eigen::Index joint = 0;
    std::size_t after  = 0;
};

void report_largest_turn(const joint_turn &largest, std::size_t waypoints)
{
    if(largest.degrees <= reported_joint_turn_degrees)
        return;

    const std::string from = largest.after == 0u ? std::string("the starting configuration") : std::format("waypoint {}", largest.after);
    spdlog::warn("praxis: joint {} turns {:.1f} degrees between {} and waypoint {} of {}, so the motion between them may leave the path the waypoints sample", largest.joint + 1,
                 largest.degrees, from, largest.after + 1u, waypoints);
}

expected<std::vector<joint_vector>, refusal> resolved_waypoints(const motion_ops &motion, const kinematics &solver, std::span<const transform> waypoints, const joint_vector &j0)
{
    std::vector<joint_vector> resolved;
    resolved.reserve(waypoints.size() + 1u);
    if(waypoints.size() == 1u)
        resolved.push_back(j0);

    joint_vector seed = j0;
    joint_turn largest;
    for(std::size_t i = 0; i < waypoints.size(); ++i)
    {
        const expected<joint_vector, refusal> reached = motion.task_space_pose(solver, waypoints[i], seed);
        if(!reached)
        {
            spdlog::error("praxis: waypoint {} of {} resolved to no configuration, so no task-space motion is produced", i + 1u, waypoints.size());
            return unexpected(reached.error());
        }

        std::optional<joint_vector> turned = turned_within_limits(solver.space_chain(), *reached, seed);
        if(!turned)
        {
            spdlog::error("praxis: waypoint {} of {} resolved to a configuration no whole turn of its joints brings inside the joint limits, so no task-space motion is produced",
                          i + 1u, waypoints.size());
            return unexpected(refusal::no_solution);
        }

        Eigen::Index joint   = 0;
        const double degrees = to_degrees((*turned - seed).cwiseAbs().maxCoeff(&joint));
        if(degrees > largest.degrees)
            largest = joint_turn{degrees, joint, i};

        seed = std::move(*turned);
        resolved.push_back(seed);
    }
    report_largest_turn(largest, waypoints.size());

    return resolved;
}

void report_via_point_step_refusal(refusal reason, std::size_t resolved)
{
    if(reason == refusal::not_implemented)
        spdlog::warn("praxis: 'trajectory.via_point_schedule' or 'trajectory.joint_space_via_points' is not implemented, so no task-space motion is produced");
    else
        spdlog::error("praxis: the via-point step or slot refused {} resolved waypoints, so no task-space motion is produced", resolved);
}

}

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> task_space_waypoints(const trajectory::trajectory_ops &trajectory, const motion_ops &motion,
                                                                                          const kinematics &solver, std::span<const transform> waypoints, const joint_vector &j0,
                                                                                          const joint_limits &limits)
{
    if(waypoints.empty())
    {
        spdlog::error("praxis: a task-space motion needs at least one waypoint and none was given");
        return unexpected(refusal::unsupported_input);
    }

    const expected<std::vector<joint_vector>, refusal> resolved = resolved_waypoints(motion, solver, waypoints, j0);
    if(!resolved)
        return unexpected(resolved.error());

    auto joined = via_point_motion(trajectory, *resolved, j0, limits);
    if(!joined)
    {
        report_via_point_step_refusal(joined.error(), resolved->size());
        return unexpected(joined.error());
    }

    return std::move(*joined);
}

}
