#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_BENT_SOLVER_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_BENT_SOLVER_H

#include "praxis/manipulator/baseline/robot.h"
#include "praxis/manipulator/baseline/modeling.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <meios/model.h>

#include <vector>

// Bindings for the three slots that run a solve, and two chains a shared forward map cannot be taken
// over. Each answers a pose the row can hold against the one that was asked for: that pose reached
// along another branch, that pose displaced by a stated amount, or no pose at all.
namespace praxis::fixture {

using namespace manipulator;

// How far from the pose it was asked for a displaced solve lands. The two halves are independent: the
// angular half turns the request about x and the linear half moves it along x, so the pose residual
// between the two is exactly the first in radians and exactly the second in metres.
inline double off_target_radians = 0.0;
inline double off_target_metres  = 0.0;

inline transform off_target(const transform &desired)
{
    const rotation further(Eigen::AngleAxisd(off_target_radians, Eigen::Vector3d::UnitX()).toRotationMatrix());
    transform moved = desired;

    moved.block<3, 3>(0, 0) = rotation(desired.block<3, 3>(0, 0) * further);
    moved(0, 3) += off_target_metres;

    return moved;
}

inline expected<void, refusal> solves_for_a_displaced_target(const forward_kinematics_ops &forward, const differential_kinematics_ops &differential, const screw_chain &chain,
                                                             const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer)
{
    return inverse_kinematics(forward, differential, chain, off_target(desired), j0, parameters, answer);
}

// The search started from the seed reflected through the origin of joint space, which reaches the pose
// it was asked for along a branch the seed itself does not lead to.
inline expected<void, refusal> solves_from_a_reflected_seed(const forward_kinematics_ops &forward, const differential_kinematics_ops &differential, const screw_chain &chain,
                                                            const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer)
{
    return inverse_kinematics(forward, differential, chain, desired, joint_vector(-j0), parameters, answer);
}

inline expected<void, refusal> answers_one_joint_too_many(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const transform &,
                                                          const joint_vector &j0, const solver_parameters &, ik_result &answer)
{
    answer.solutions.emplace_back(joint_vector::Zero(j0.size() + 1));

    return {};
}

inline expected<joint_vector, refusal> solve_pose_for_a_displaced_target(const kinematics &solver, const rigid_motion::frame_ops &frames, const transform &tool_pose,
                                                                         const joint_vector &j0, const transform &tool_offset)
{
    return ik_solve_pose(solver, frames, off_target(tool_pose), j0, tool_offset);
}

inline expected<joint_vector, refusal> solve_flange_pose_for_a_displaced_target(const kinematics &solver, const transform &flange_pose, const joint_vector &j0)
{
    return ik_solve_flange_pose(solver, off_target(flange_pose), j0);
}

inline expected<joint_vector, refusal> solve_pose_from_a_reflected_seed(const kinematics &solver, const rigid_motion::frame_ops &frames, const transform &tool_pose,
                                                                        const joint_vector &j0, const transform &tool_offset)
{
    return ik_solve_pose(solver, frames, tool_pose, joint_vector(-j0), tool_offset);
}

inline expected<joint_vector, refusal> solve_flange_pose_from_a_reflected_seed(const kinematics &solver, const transform &flange_pose, const joint_vector &j0)
{
    return ik_solve_flange_pose(solver, flange_pose, joint_vector(-j0));
}

inline expected<joint_vector, refusal> answers_one_joint_too_many_at_the_tool(const kinematics &, const rigid_motion::frame_ops &, const transform &, const joint_vector &j0,
                                                                              const transform &)
{
    return joint_vector(joint_vector::Zero(j0.size() + 1));
}

inline expected<joint_vector, refusal> answers_one_joint_too_many_at_the_flange(const kinematics &, const transform &, const joint_vector &j0)
{
    return joint_vector(joint_vector::Zero(j0.size() + 1));
}

inline expected<joint_vector, refusal> declines_at_the_tool(const kinematics &, const rigid_motion::frame_ops &, const transform &, const joint_vector &, const transform &)
{
    return unexpected(refusal::no_solution);
}

inline expected<joint_vector, refusal> declines_at_the_tool_for_another_reason(const kinematics &, const rigid_motion::frame_ops &, const transform &, const joint_vector &,
                                                                               const transform &)
{
    return unexpected(refusal::unsupported_input);
}

inline expected<joint_vector, refusal> declines_at_the_flange(const kinematics &, const transform &, const joint_vector &)
{
    return unexpected(refusal::no_solution);
}

inline expected<joint_vector, refusal> declines_at_the_flange_for_another_reason(const kinematics &, const transform &, const joint_vector &)
{
    return unexpected(refusal::unsupported_input);
}

// A home pose no rigid motion, so the shared forward map declines at every configuration while the
// chain still carries one screw per joint and its width matches the reference's.
inline expected<screw_chain, refusal> a_home_that_is_no_rigid_motion(const meios::model<> &model)
{
    expected<screw_chain, refusal> derived = build_chain(model);
    if(!derived)
        return derived;

    derived->home(0, 0) += 1.0;

    return derived;
}

}

#endif
