#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_EVALUATION_TABLES_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_EVALUATION_TABLES_H

#include "praxis/manipulator/robot.h"
#include "praxis/manipulator/motion.h"
#include "praxis/manipulator/modeling.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/task_trajectory.h"

#include "praxis/optimization/solver.h"

#include "praxis/evaluation/comparators.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <limits>
#include <cstddef>
#include <optional>
#include <algorithm>

namespace praxis::manipulator {

struct evaluation_case;

// The bound the two rows whose answer is a Jacobian accumulated along a chain of up to eight screws
// are judged at, in the dimensionless unit an element-wise difference carries.
inline constexpr double accumulated_element_wise_tolerance = 1.0e-11;

// The bounds the three rows whose answer is a pose accumulated along a chain of up to eight screws are
// judged at: a rotation in radians and a distance in meters, never added to one another.
inline constexpr double accumulated_pose_tolerance_radians = 1.0e-5;
inline constexpr double accumulated_pose_tolerance_meters  = 1.0e-4;

// The bounds the robot and motion rows whose answer is a configuration read back through a forward
// map and held against the pose it was asked for are judged at: a rotation in radians and a distance
// in meters, never added to one another. Each stands above the stopping criterion `solver_parameters`
// publishes.
inline constexpr double solved_pose_tolerance_radians = 1.0e-6;
inline constexpr double solved_pose_tolerance_meters  = 1.0e-6;

// The bounds the inverse kinematics row is judged at: a rotation in radians and a distance in meters.
inline constexpr double inverse_kinematics_tolerance_radians = 1.0e-6;
inline constexpr double inverse_kinematics_tolerance_meters  = 1.0e-5;

// The bounds the row judging a bound solver through the inverse kinematics adapters is judged at: a
// rotation in radians and a distance in meters.
inline constexpr double solver_route_tolerance_radians = 1.0e-6;
inline constexpr double solver_route_tolerance_meters  = 1.0e-5;

// The bound the row whose answer is a prepared task-space motion is judged at, in the dimensionless
// unit an element-wise difference carries. One sample holds a configuration, a rate and an
// acceleration, which fold to whichever of the three stands loosest, the acceleration taken as a
// fraction of the larger of one and either side's magnitude.
inline constexpr double prepared_motion_element_wise_tolerance = 1.0e-7;

// The run the bound above was measured over. It is not known to hold over a longer one: the shortest
// segment a run draws has no floor.
inline constexpr std::size_t prepared_motion_measured_to_cases = 2000;

const evaluation::capability_evaluations<robot_ops> &robot_evaluations();
const evaluation::capability_evaluations<motion_ops> &motion_evaluations();
const evaluation::capability_evaluations<modeling_ops> &modeling_evaluations();
const evaluation::capability_evaluations<task_trajectory_ops> &task_trajectory_evaluations();
const evaluation::capability_evaluations<forward_kinematics_ops> &forward_kinematics_evaluations();
const evaluation::capability_evaluations<differential_kinematics_ops> &differential_kinematics_evaluations();
const evaluation::capability_evaluations<inverse_kinematics_ops> &inverse_kinematics_evaluations();
const evaluation::capability_evaluations<optimization::solver_ops> &solver_route_evaluations();

const rigid_motion::screw_ops &shared_screw();
const rigid_motion::frame_ops &shared_frames();
const forward_kinematics_ops &shared_forward();
const differential_kinematics_ops &shared_differential();
const inverse_kinematics_ops &shared_inverse();
const optimization::solver_ops &shared_solver();
const motion_ops &shared_motion();
const trajectory::trajectory_ops &shared_trajectory();

// A target the chain stands at when driven to a drawn configuration, so a solve that declines failed
// rather than being asked for an unreachable pose; a seed drawn on its own; and the holder, bound to
// the reference, that both sides' answers are read back through. The forward maps and the Jacobians
// are the reference's and both compared sides are handed the same pair, so each side measures its own
// slot rather than that pair.
struct solve_request
{
    kinematics reference;
    forward_kinematics_ops forward;
    differential_kinematics_ops differential;
    transform target;
    joint_vector seed;
};

std::optional<solve_request> drawn_request(evaluation::case_source &drawn, const evaluation_case &example);

// A solve that succeeds with no configuration in its answer is read as the refusal no_solution.
expected<void, refusal> answered_with_a_configuration(const expected<void, refusal> &solved, const ik_result &answer);

// A pose a chain can reach is reached by more than one configuration, so an answer is read back
// through one shared forward map before it is held against anything.
std::optional<transform> reached_by(const kinematics &reference, const ik_result &answer);

inline evaluation::case_result judged(const evaluation::residual &difference, const evaluation::tolerance_pair &allowed)
{
    return evaluation::case_result{evaluation::verdict_of(difference, allowed), difference};
}

// A case whose shared inputs could not be built. Its residual is the kind's own zero because nothing
// was measured, and nothing reads it.
inline evaluation::case_result unusable(evaluation::residual_kind kind)
{
    return evaluation::case_result{evaluation::agreement::unusable, evaluation::residual{kind, 0.0, 0.0}};
}

// A difference no residual measures: two sequences of different length, two chains of different
// width, an answer the forward map cannot read back. Its magnitude is unbounded in the kind's own unit.
inline evaluation::case_result categorically_differed(evaluation::residual_kind kind)
{
    constexpr double unbounded = std::numeric_limits<double>::infinity();

    return evaluation::case_result{evaluation::agreement::differed, evaluation::residual{kind, unbounded, unbounded}};
}

// The facility's refusal policy, held identical to `agreed_or_refused` for the value shapes and the
// verdicts that helper cannot serve. Absent where both sides answered. Two that declined the same
// input agree about it; two that declined for different reasons are an outcome of their own; exactly
// one declining is a third. Each carries a value-initialized residual: no refusal becomes a number.
template<typename T>
std::optional<evaluation::case_result> refusal_outcome(const expected<T, refusal> &first, const expected<T, refusal> &second)
{
    if(first.has_value() && second.has_value())
        return std::nullopt;
    if(first.has_value() != second.has_value())
        return evaluation::case_result{evaluation::agreement::one_refused, evaluation::residual{}};

    return evaluation::case_result{first.error() == second.error() ? evaluation::agreement::both_refused : evaluation::agreement::refused_differently, evaluation::residual{}};
}

// Each side's answer, already read forward through the map both were handed, held against the pose
// that was asked for; the case takes the worse of the two halves and never holds one answer against
// the other. An answer the map cannot read back differs by more than any bound.
inline evaluation::case_result reaching_what_was_asked(const std::optional<transform> &here, const std::optional<transform> &there, const transform &asked_for,
                                                       const evaluation::tolerance_pair &allowed)
{
    if(!here || !there)
        return categorically_differed(evaluation::residual_kind::pose);

    const evaluation::residual from_here  = evaluation::pose_residual(*here, asked_for);
    const evaluation::residual from_there = evaluation::pose_residual(*there, asked_for);

    return judged(evaluation::residual{evaluation::residual_kind::pose, std::max(from_here.magnitude, from_there.magnitude),
                                       std::max(from_here.linear_error_metres, from_there.linear_error_metres)},
                  allowed);
}

// The three slots whose return types no shipped residual folds over: a sequence of screw axes, a
// solve that answers nothing and writes into a result the caller owns, and a whole chain; and a
// solver, judged by the solve it drives through an inverse kinematics adapter.
evaluation::case_result compare_build_chain(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed);
evaluation::case_result compare_inverse_kinematics(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed);
evaluation::case_result compare_body_screws_from_space(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed);
evaluation::case_result compare_solver_route(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed);

}

#endif
