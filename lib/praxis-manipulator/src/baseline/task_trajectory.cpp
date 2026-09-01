#include "praxis/manipulator/baseline/motion.h"
#include "praxis/manipulator/baseline/task_trajectory.h"

#include "praxis/trajectory/baseline/trajectory.h"

#include <spdlog/spdlog.h>

#include <memory>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::manipulator {

namespace {

expected<std::vector<joint_vector>, refusal> resolved_waypoints(const kinematics &solver, std::span<const transform> waypoints, const joint_vector &j0)
{
    std::vector<joint_vector> resolved;
    resolved.reserve(waypoints.size() + 1u);
    if(waypoints.size() == 1u)
        resolved.push_back(j0);

    joint_vector seed = j0;
    for(std::size_t i = 0; i < waypoints.size(); ++i)
    {
        const expected<joint_vector, refusal> reached = task_space_pose(solver, waypoints[i], seed);
        if(!reached)
        {
            spdlog::error("praxis: waypoint {} of {} resolved to no configuration, so no task-space motion is produced", i + 1u, waypoints.size());
            return unexpected(reached.error());
        }

        seed = *reached;
        resolved.push_back(seed);
    }

    return resolved;
}

}

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> task_space_waypoints(const kinematics &solver, std::span<const transform> waypoints, const joint_vector &j0,
                                                                                          const joint_limits &limits)
{
    if(waypoints.empty())
    {
        spdlog::error("praxis: a task-space motion needs at least one waypoint and none was given");
        return unexpected(refusal::unsupported_input);
    }

    const expected<std::vector<joint_vector>, refusal> resolved = resolved_waypoints(solver, waypoints, j0);
    if(!resolved)
        return unexpected(resolved.error());

    auto motion = trajectory::joint_space_waypoints(*resolved, j0, limits);
    if(!motion)
    {
        spdlog::error("praxis: the configuration-space factory refused {} resolved waypoints, so no task-space motion is produced", resolved->size());
        return unexpected(motion.error());
    }

    return std::move(*motion);
}

}
