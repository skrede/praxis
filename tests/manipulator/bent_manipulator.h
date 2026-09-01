#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_BENT_MANIPULATOR_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_BENT_MANIPULATOR_H

#include "bend_kinds.h"
#include "bent_motions.h"

#include "praxis/manipulator/baseline/robot.h"
#include "praxis/manipulator/baseline/modeling.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/capabilities.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <span>
#include <vector>

// Every binding below answers the reference displaced in the one quantity the row that compares it
// measures, by the entry of `bend_by` its own residual reads.
namespace praxis::fixture {

using namespace manipulator;

// Both halves at once: the angular half turns the rotation block about x and the linear half moves
// the origin along it, so a run setting one entry and leaving the other at zero moves that half alone.
inline transform displaced(const transform &pose)
{
    const rotation further(Eigen::AngleAxisd(bent_by(bent::pose_radians), Eigen::Vector3d::UnitX()).toRotationMatrix());
    transform moved = pose;

    moved.block<3, 3>(0, 0) = further * pose.block<3, 3>(0, 0);
    moved(0, 3) += bent_by(bent::pose_metres);

    return moved;
}

inline expected<transform, refusal> displaced_forward(const transform &m, std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    const expected<transform, refusal> reached = forward_kinematics(m, space_screws, theta);
    if(!reached)
        return reached;

    return displaced(reached.value());
}

inline expected<transform, refusal> displaced_body_forward(const rigid_motion::frame_ops &frames, const transform &m, std::span<const screw_axis> body_screws, const joint_vector &theta)
{
    const expected<transform, refusal> reached = body_forward_kinematics(frames, m, body_screws, theta);
    if(!reached)
        return reached;

    return displaced(reached.value());
}

inline expected<jacobian, refusal> moved_space_jacobian(std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    expected<jacobian, refusal> columns = space_jacobian(space_screws, theta);
    if(!columns)
        return columns;

    columns.value()(0, 0) += bent_by(bent::element_wise);

    return columns;
}

inline expected<jacobian, refusal> moved_body_jacobian(std::span<const screw_axis> body_screws, const joint_vector &theta)
{
    expected<jacobian, refusal> columns = body_jacobian(body_screws, theta);
    if(!columns)
        return columns;

    columns.value()(0, 0) += bent_by(bent::element_wise);

    return columns;
}

inline transform displaced_tool_pose(const transform &flange_pose, const transform &tool_offset)
{
    return displaced(tool_pose_from_flange_pose(flange_pose, tool_offset));
}

inline transform displaced_flange_pose(const rigid_motion::frame_ops &frames, const transform &tool_pose, const transform &tool_offset)
{
    return displaced(flange_pose_from_tool_pose(frames, tool_pose, tool_offset));
}

inline Eigen::Vector3d moved_position(const transform &pose)
{
    return Eigen::Vector3d(position_from_pose(pose) + bent_by(bent::element_wise) * Eigen::Vector3d::UnitX());
}

inline rotation turned_orientation(const transform &pose)
{
    const rotation further(Eigen::AngleAxisd(bent_by(bent::geodesic), Eigen::Vector3d::UnitX()).toRotationMatrix());

    return rotation(further * orientation_from_pose(pose));
}

inline expected<joint_vector, refusal> nudged_solve_pose(const kinematics &solver, const rigid_motion::frame_ops &frames, const transform &tool_pose, const joint_vector &j0,
                                                         const transform &tool_offset)
{
    const expected<joint_vector, refusal> answer = ik_solve_pose(solver, frames, tool_pose, j0, tool_offset);
    if(!answer)
        return answer;

    return nudged(answer.value());
}

inline expected<joint_vector, refusal> nudged_solve_flange_pose(const kinematics &solver, const transform &flange_pose, const joint_vector &j0)
{
    const expected<joint_vector, refusal> answer = ik_solve_flange_pose(solver, flange_pose, j0);
    if(!answer)
        return answer;

    return nudged(answer.value());
}

inline expected<std::vector<screw_axis>, refusal> displaced_body_screws(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &m,
                                                                        std::span<const screw_axis> space_screws)
{
    expected<std::vector<screw_axis>, refusal> derived = body_screws_from_space(screw, frames, m, space_screws);
    if(!derived)
        return derived;

    derived->front()(0) += bent_by(bent::element_wise);

    return derived;
}

inline expected<void, refusal> nudged_iterative_solve(const forward_kinematics_ops &forward, const differential_kinematics_ops &differential, const screw_chain &chain,
                                                      const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer)
{
    const expected<void, refusal> solved = inverse_kinematics(forward, differential, chain, desired, j0, parameters, answer);
    if(!solved)
        return solved;

    for(joint_vector &solution : answer.solutions)
        solution = nudged(solution);

    return solved;
}

inline expected<screw_chain, refusal> displaced_chain(const meios::model<> &model)
{
    expected<screw_chain, refusal> derived = build_chain(model);
    if(!derived)
        return derived;

    derived->home = displaced(derived->home);

    return derived;
}

inline capabilities bent_everywhere()
{
    capabilities arm = baseline();

    arm.fk.forward_kinematics            = &displaced_forward;
    arm.fk.body_forward_kinematics       = &displaced_body_forward;
    arm.fk.body_screws_from_space        = &displaced_body_screws;
    arm.dk.space_jacobian                = &moved_space_jacobian;
    arm.dk.body_jacobian                 = &moved_body_jacobian;
    arm.ik.inverse_kinematics            = &nudged_iterative_solve;
    arm.modeling.build_chain             = &displaced_chain;
    arm.robot.tool_pose_from_flange_pose = &displaced_tool_pose;
    arm.robot.flange_pose_from_tool_pose = &displaced_flange_pose;
    arm.robot.position_from_pose         = &moved_position;
    arm.robot.orientation_from_pose      = &turned_orientation;
    arm.robot.ik_solve_pose              = &nudged_solve_pose;
    arm.robot.ik_solve_flange_pose       = &nudged_solve_flange_pose;
    arm.motion.task_space_pose           = &nudged_task_space_pose;
    arm.motion.task_space_screw          = &nudged_task_space_screw;
    arm.motion.tool_frame_displace       = &nudged_tool_frame_displace;
    arm.trajectory.task_space_waypoints  = &displaced_task_space_waypoints;

    return arm;
}

}

#endif
