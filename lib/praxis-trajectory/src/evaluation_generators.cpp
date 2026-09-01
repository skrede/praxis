#include "evaluation_cases.h"
#include "evaluation_tables.h"
#include "evaluation_driving.h"

#include "praxis/trajectory/slots.h"

#include <span>
#include <array>
#include <memory>
#include <cstddef>
#include <optional>

namespace praxis::trajectory {

namespace {

using pose_waypoint_factory = expected<std::unique_ptr<pose_trajectory_generator>, refusal> (*)(std::span<const transform> waypoints, const transform &seed, double max_linear_speed,
                                                                                                double max_angular_speed);

const trajectory_ops &trajectories_of(const void *value)
{
    return *static_cast<const trajectory_ops *>(value);
}

const pose_trajectory_ops &pose_trajectories_of(const void *value)
{
    return *static_cast<const pose_trajectory_ops *>(value);
}

constexpr evaluation::residual_kind configuration_kind = evaluation::residual_kind::element_wise;
constexpr evaluation::residual_kind pose_kind          = evaluation::residual_kind::pose;

evaluation::case_result compare_joint_space_waypoints(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const joint_waypoint_case example = drawn_joint_waypoint_case(drawn);
    if(!advances_at_every_row(example.seed, example.waypoints))
        return unusable_input(configuration_kind);

    const expected<std::unique_ptr<trajectory_generator>, refusal> held  = trajectories_of(first).joint_space_waypoints(example.waypoints, example.seed, example.limits);
    const expected<std::unique_ptr<trajectory_generator>, refusal> other = trajectories_of(second).joint_space_waypoints(example.waypoints, example.seed, example.limits);
    if(const std::optional<evaluation::case_result> refused = refusal_outcome(held, other))
        return *refused;
    if(!both_named_a_motion(*held, *other))
        return differing(configuration_kind);

    return driven(**held, **other, configuration_kind, allowed);
}

evaluation::case_result over_pose_waypoints(pose_waypoint_factory held_by, pose_waypoint_factory bound_by, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const pose_waypoint_case example = drawn_pose_waypoint_case(drawn);
    if(!advances_at_every_row(example.seed, example.waypoints))
        return unusable_input(pose_kind);

    const expected<std::unique_ptr<pose_trajectory_generator>, refusal> held  = held_by(example.waypoints, example.seed, example.max_linear_speed, example.max_angular_speed);
    const expected<std::unique_ptr<pose_trajectory_generator>, refusal> other = bound_by(example.waypoints, example.seed, example.max_linear_speed, example.max_angular_speed);
    if(const std::optional<evaluation::case_result> refused = refusal_outcome(held, other))
        return *refused;
    if(!both_named_a_motion(*held, *other))
        return differing(pose_kind);

    return driven(**held, **other, pose_kind, allowed);
}

evaluation::case_result compare_decoupled_pose_waypoints(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    return over_pose_waypoints(pose_trajectories_of(first).decoupled_pose_waypoints, pose_trajectories_of(second).decoupled_pose_waypoints, drawn, allowed);
}

evaluation::case_result compare_screw_pose_waypoints(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    return over_pose_waypoints(pose_trajectories_of(first).screw_pose_waypoints, pose_trajectories_of(second).screw_pose_waypoints, drawn, allowed);
}

// The rows are in the enumerator order of their own slot enumeration, and each name is spelled
// exactly as the descriptor table spells it.
constexpr evaluation::tolerance_pair driven_pose_allowance{driven_pose_tolerance_radians, driven_pose_tolerance_metres};
constexpr evaluation::tolerance_pair driven_configuration_allowance{driven_configuration_tolerance, driven_configuration_tolerance};

constexpr std::array pose_trajectory_table{
        evaluation::slot_evaluation{"pose_trajectory.decoupled_pose_waypoints", pose_kind, driven_pose_allowance, &compare_decoupled_pose_waypoints},
        evaluation::slot_evaluation{"pose_trajectory.screw_pose_waypoints", pose_kind, driven_pose_allowance, &compare_screw_pose_waypoints},
};

constexpr std::array trajectory_table{
        evaluation::slot_evaluation{"trajectory.joint_space_waypoints", configuration_kind, driven_configuration_allowance, &compare_joint_space_waypoints,
                                    driven_configuration_measured_to_cases},
};

static_assert(pose_trajectory_table.size() == static_cast<std::size_t>(pose_trajectory_slot::count));
static_assert(trajectory_table.size() == static_cast<std::size_t>(trajectory_slot::count));

constexpr evaluation::capability_evaluations<pose_trajectory_ops> evaluated_pose_trajectories{"trajectory", pose_trajectory_table};
constexpr evaluation::capability_evaluations<trajectory_ops> evaluated_trajectories{"trajectory", trajectory_table};

}

const evaluation::capability_evaluations<pose_trajectory_ops> &pose_trajectory_evaluations()
{
    return evaluated_pose_trajectories;
}

const evaluation::capability_evaluations<trajectory_ops> &trajectory_evaluations()
{
    return evaluated_trajectories;
}

}
