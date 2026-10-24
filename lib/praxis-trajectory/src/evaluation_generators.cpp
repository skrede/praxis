#include "evaluation_cases.h"
#include "evaluation_tables.h"
#include "evaluation_driving.h"

#include "praxis/trajectory/slots.h"

#include "praxis/evaluation/comparators.h"

#include <span>
#include <array>
#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <optional>
#include <algorithm>

namespace praxis::trajectory {

namespace {

using pose_waypoint_factory = decltype(pose_trajectory_ops::decoupled_pose_waypoints);

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

// The times a run's acceleration steps at: every via point but the first and the last.
std::vector<double> interior_times(std::span<const via_point> via_points)
{
    std::vector<double> times;
    for(std::size_t k = 1u; k + 1u < via_points.size(); ++k)
        times.push_back(via_points[k].time);

    return times;
}

evaluation::case_result compare_joint_space_via_points(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const via_point_case example = drawn_via_point_case(drawn);

    const expected<std::unique_ptr<trajectory_generator>, refusal> held  = trajectories_of(first).joint_space_via_points(example.via_points);
    const expected<std::unique_ptr<trajectory_generator>, refusal> other = trajectories_of(second).joint_space_via_points(example.via_points);
    if(const std::optional<evaluation::case_result> refused = refusal_outcome(held, other))
        return *refused;
    if(!both_named_a_motion(*held, *other))
        return differing(configuration_kind);

    return driven(**held, **other, configuration_kind, allowed, interior_times(example.via_points));
}

// Each time over the larger of one and the second side's magnitude, each position and velocity by
// its difference, folded to the loosest.
evaluation::residual schedule_residual(const std::vector<via_point> &held, const std::vector<via_point> &other)
{
    double worst = 0.0;
    for(std::size_t k = 0u; k < held.size(); ++k)
    {
        const via_point &first  = held[k];
        const via_point &second = other[k];
        if(!std::isfinite(first.time) || !std::isfinite(second.time) || !first.position.allFinite() || !first.velocity.allFinite() || !second.position.allFinite() ||
           !second.velocity.allFinite() || first.position.size() != second.position.size() || first.velocity.size() != second.velocity.size())
            return differing(configuration_kind).difference;

        worst = std::max({worst, std::abs(first.time - second.time) / std::max(1.0, std::abs(second.time)), evaluation::element_wise_residual(first.position, second.position).magnitude,
                          evaluation::element_wise_residual(first.velocity, second.velocity).magnitude});
    }

    return evaluation::residual{configuration_kind, worst, 0.0};
}

evaluation::case_result compare_via_point_schedule(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const joint_waypoint_case example = drawn_joint_waypoint_case(drawn);
    if(!advances_at_every_row(example.seed, example.waypoints))
        return unusable_input(configuration_kind);

    const expected<std::vector<via_point>, refusal> held  = trajectories_of(first).via_point_schedule(example.waypoints, example.seed, example.limits);
    const expected<std::vector<via_point>, refusal> other = trajectories_of(second).via_point_schedule(example.waypoints, example.seed, example.limits);
    if(const std::optional<evaluation::case_result> refused = refusal_outcome(held, other))
        return *refused;
    if(held->size() != other->size())
        return differing(configuration_kind);

    return judged(schedule_residual(*held, *other), allowed);
}

evaluation::case_result over_pose_waypoints(pose_waypoint_factory held_by, pose_waypoint_factory bound_by, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const pose_waypoint_case example = drawn_pose_waypoint_case(drawn);
    if(!advances_at_every_row(example.seed, example.waypoints))
        return unusable_input(pose_kind);

    const expected<std::unique_ptr<pose_trajectory_generator>, refusal> held =
            held_by(shared_screw(), shared_frames(), shared_paths(), shared_time_scalings(), example.waypoints, example.seed, example.max_linear_speed, example.max_angular_speed);
    const expected<std::unique_ptr<pose_trajectory_generator>, refusal> other =
            bound_by(shared_screw(), shared_frames(), shared_paths(), shared_time_scalings(), example.waypoints, example.seed, example.max_linear_speed, example.max_angular_speed);
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
constexpr evaluation::tolerance_pair driven_pose_allowance{driven_pose_tolerance_radians, driven_pose_tolerance_meters};
constexpr evaluation::tolerance_pair via_point_allowance{via_point_run_tolerance, via_point_run_tolerance};
constexpr evaluation::tolerance_pair schedule_allowance{via_point_schedule_tolerance, via_point_schedule_tolerance};

constexpr std::array pose_trajectory_table{
        evaluation::slot_evaluation{"pose_trajectory.decoupled_pose_waypoints", pose_kind, driven_pose_allowance, &compare_decoupled_pose_waypoints},
        evaluation::slot_evaluation{"pose_trajectory.screw_pose_waypoints", pose_kind, driven_pose_allowance, &compare_screw_pose_waypoints},
};

constexpr std::array trajectory_table{
        evaluation::slot_evaluation{"trajectory.joint_space_via_points", configuration_kind, via_point_allowance, &compare_joint_space_via_points},
        evaluation::slot_evaluation{"trajectory.via_point_schedule", configuration_kind, schedule_allowance, &compare_via_point_schedule, via_point_schedule_measured_to_cases},
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
