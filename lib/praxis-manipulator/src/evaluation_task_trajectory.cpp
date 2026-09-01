#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/manipulator/capabilities.h"

#include "praxis/trajectory/evaluation.h"

#include <array>
#include <memory>
#include <vector>
#include <cstddef>
#include <numbers>
#include <utility>
#include <optional>

namespace praxis::manipulator {

namespace {

constexpr evaluation::residual_kind waypoint_kind = evaluation::residual_kind::element_wise;

// How many poses one drawn via-point set carries, and how far in each joint the chain is driven
// between one and the next. The steps are short so a factory chaining each solve's answer into the
// next one's seed is asked for a motion the chain can make, and a refusal in the run is that factory
// failing rather than a waypoint outside the workspace.
constexpr std::size_t waypoints_per_case = 3u;
constexpr double waypoint_step_radians   = 0.05;

const task_trajectory_ops &trajectories_of(const void *value)
{
    return *static_cast<const task_trajectory_ops *>(value);
}

// A solver over the drawn chain, a sequence of poses that chain stands at when driven, and the
// configuration the run starts from.
struct waypoint_case
{
    kinematics solver;
    std::vector<transform> waypoints;
    joint_vector seed;
};

// Drawn from one source in this order and no other: a solve case, then one short step per joint per
// waypoint. Each pose is the chain driven to the configuration those steps reach.
std::optional<waypoint_case> drawn_waypoints(evaluation::case_source &drawn)
{
    std::optional<solve_case> solved = drawn_solve(drawn);
    if(!solved)
        return std::nullopt;

    joint_vector standing = solved->standing;
    std::vector<transform> waypoints;
    waypoints.reserve(waypoints_per_case);
    for(std::size_t at = 0; at < waypoints_per_case; ++at)
    {
        standing += drawn_joints(drawn, static_cast<std::size_t>(standing.size())) * (waypoint_step_radians / std::numbers::pi);

        const std::optional<transform> reached = reached_by(*solved, standing);
        if(!reached)
            return std::nullopt;

        waypoints.push_back(*reached);
    }

    return waypoint_case{std::move(solved->solver), std::move(waypoints), solved->standing};
}

// The two prepared motions are compared by what they compute, through the comparison the trajectory
// extension publishes for the generator type this factory answers. Neither pointer is read until the
// refusal policy and the null-answer test have both run.
evaluation::case_result compare_task_space_waypoints(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const std::optional<waypoint_case> example = drawn_waypoints(drawn);
    if(!example)
        return unusable(waypoint_kind);

    const joint_limits &bounds = example->solver.space_chain().limits;
    const expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> held =
            trajectories_of(first).task_space_waypoints(example->solver, example->waypoints, example->seed, bounds);
    const expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> against =
            trajectories_of(second).task_space_waypoints(example->solver, example->waypoints, example->seed, bounds);
    if(const std::optional<evaluation::case_result> refused = refusal_outcome(held, against))
        return *refused;
    if(*held == nullptr || *against == nullptr)
        return categorically_differed(waypoint_kind);

    return trajectory::driven(**held, **against, waypoint_kind, allowed);
}

// The one row is in the enumerator order of task_trajectory_slot, and its name is spelled exactly as
// the descriptor table spells it.
constexpr std::array task_trajectory_table{
        evaluation::slot_evaluation{"trajectory.task_space_waypoints", waypoint_kind,
                                    evaluation::tolerance_pair{prepared_motion_element_wise_tolerance, prepared_motion_element_wise_tolerance}, &compare_task_space_waypoints,
                                    prepared_motion_measured_to_cases},
};

static_assert(task_trajectory_table.size() == static_cast<std::size_t>(task_trajectory_slot::count));

constexpr evaluation::capability_evaluations<task_trajectory_ops> evaluated_task_trajectories{"manipulator", task_trajectory_table};

}

const evaluation::capability_evaluations<task_trajectory_ops> &task_trajectory_evaluations()
{
    return evaluated_task_trajectories;
}

}
