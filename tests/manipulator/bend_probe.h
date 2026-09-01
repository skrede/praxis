#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_BEND_PROBE_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_BEND_PROBE_H

#include "evaluation_cases.h"
#include "described_slots.h"

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/trajectory/trajectory.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/evaluation/generation.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>

#include <meios/model.h>

#include <array>
#include <vector>
#include <memory>
#include <cstddef>
#include <cstdint>

// One drawn case put through every compared slot on two aggregates, asking of each row whether the
// two answered differently at all. Nothing here reads a tolerance: what a bend has to clear is the
// ledger's question, and whether it reaches the answer is this one.
namespace praxis::fixture {

using namespace evaluation;

constexpr std::uint64_t recorded_seed = 0xC0FFEEu;
constexpr std::size_t probed_cases    = 32u;
constexpr std::size_t every_row       = compared_slots;

// A step short enough that the solve the via-point factory chains from the run's own start
// configuration reaches the pose one step on from it.
constexpr double waypoint_step_radians = 0.05;

bool apart(const Eigen::Ref<const Eigen::MatrixXd> &held, const Eigen::Ref<const Eigen::MatrixXd> &against)
{
    return held.rows() != against.rows() || held.cols() != against.cols() || (held - against).cwiseAbs().maxCoeff() > 0.0;
}

// Two prepared motions stand apart where their spans do, where one names a motion and the other does
// not, or where the configurations they name at the middle of the run are not the same value.
bool apart(const std::unique_ptr<trajectory::trajectory_generator> &held, const std::unique_ptr<trajectory::trajectory_generator> &against)
{
    if(held == nullptr || against == nullptr)
        return (held == nullptr) != (against == nullptr);
    if(held->duration() != against->duration())
        return true;

    const expected<trajectory::trajectory_sample, refusal> here  = held->sample(0.5 * held->duration());
    const expected<trajectory::trajectory_sample, refusal> there = against->sample(0.5 * against->duration());
    if(!here || !there)
        return here.has_value() != there.has_value();

    return apart(here->position, there->position);
}

bool apart(const std::vector<screw_axis> &held, const std::vector<screw_axis> &against)
{
    if(held.size() != against.size())
        return true;

    for(std::size_t axis = 0; axis < held.size(); ++axis)
        if(apart(held[axis], against[axis]))
            return true;

    return false;
}

// Both sides answered and their answers are not the same value. A pair that declined tells this file
// nothing, because a refusal is not an answer a bend could have moved.
template<typename T>
bool answers_apart(const expected<T, refusal> &held, const expected<T, refusal> &against)
{
    return held.has_value() && against.has_value() && apart(*held, *against);
}

const rigid_motion::capabilities &shared_motions()
{
    static const rigid_motion::capabilities motions = rigid_motion::baseline();

    return motions;
}

// The forward map and the Jacobian are the held aggregate's on both solves, so the only difference
// between them is the inverse implementation each is handed.
std::vector<manipulator::joint_vector> solved_by(const manipulator::capabilities &held, const manipulator::inverse_kinematics_ops &solving, const manipulator::screw_chain &chain,
                                                 const transform &target, const manipulator::joint_vector &seed)
{
    manipulator::ik_result answer;
    static_cast<void>(solving.inverse_kinematics(held.fk, held.dk, chain, target, seed, manipulator::solver_parameters(), answer));

    return answer.solutions;
}

// One drawn case put through every compared slot on both aggregates, recording per row whether the
// two answered differently. The inputs are the ones each row's own comparison assembles.
void note_answers_apart(const manipulator::capabilities &held, const manipulator::capabilities &bent, case_source &drawn, std::array<bool, every_row> &seen)
{
    const manipulator::evaluation_case example = manipulator::drawn_case(drawn);
    const manipulator::joint_vector start      = manipulator::drawn_joints(drawn, example.chain.joint_count());
    const transform pose                       = drawn.transform_member();
    const transform offset                     = drawn.transform_member();
    const meios::model<> machine               = manipulator::drawn_model(drawn);
    const auto body                            = manipulator::body_screws_from_space(shared_motions().screw, shared_motions().frame, example.chain.home, example.chain.space_screws);
    auto composed                              = manipulator::kinematics::compose(example.chain, held.fk, held.dk, held.ik, shared_motions().screw, shared_motions().frame);

    seen[0] = seen[0] ||
            answers_apart(held.fk.forward_kinematics(example.chain.home, example.chain.space_screws, example.joints),
                          bent.fk.forward_kinematics(example.chain.home, example.chain.space_screws, example.joints));
    seen[2] = seen[2] ||
            answers_apart(held.fk.body_screws_from_space(shared_motions().screw, shared_motions().frame, example.chain.home, example.chain.space_screws),
                          bent.fk.body_screws_from_space(shared_motions().screw, shared_motions().frame, example.chain.home, example.chain.space_screws));
    seen[3] = seen[3] || answers_apart(held.dk.space_jacobian(example.chain.space_screws, example.joints), bent.dk.space_jacobian(example.chain.space_screws, example.joints));
    seen[6] = seen[6] || apart(held.robot.tool_pose_from_flange_pose(pose, offset), bent.robot.tool_pose_from_flange_pose(pose, offset));
    seen[7] = seen[7] || apart(held.robot.flange_pose_from_tool_pose(shared_motions().frame, pose, offset), bent.robot.flange_pose_from_tool_pose(shared_motions().frame, pose, offset));
    seen[8] = seen[8] || apart(held.robot.position_from_pose(pose), bent.robot.position_from_pose(pose));
    seen[9] = seen[9] || apart(held.robot.orientation_from_pose(pose), bent.robot.orientation_from_pose(pose));

    if(body)
    {
        seen[1] = seen[1] ||
                answers_apart(held.fk.body_forward_kinematics(shared_motions().frame, example.chain.home, *body, example.joints),
                              bent.fk.body_forward_kinematics(shared_motions().frame, example.chain.home, *body, example.joints));
        seen[4] = seen[4] || answers_apart(held.dk.body_jacobian(*body, example.joints), bent.dk.body_jacobian(*body, example.joints));
    }

    if(const auto derived = held.modeling.build_chain(machine); derived)
        if(const auto other = bent.modeling.build_chain(machine); other)
            seen[15] = seen[15] || apart(derived->home, other->home) || apart(derived->space_screws, other->space_screws);

    if(!composed)
        return;

    const auto reached = composed->fk_solve(example.joints);
    if(!reached)
        return;

    const std::vector<manipulator::joint_vector> here  = solved_by(held, held.ik, example.chain, *reached, start);
    const std::vector<manipulator::joint_vector> there = solved_by(held, bent.ik, example.chain, *reached, start);
    if(here.size() == there.size() && !here.empty())
        seen[5] = seen[5] || apart(here.front(), there.front());

    const transform tool                = *reached * offset;
    const Eigen::Vector3d direction     = drawn.unit_direction();
    const Eigen::Vector3d point         = drawn.position_metres();
    const Eigen::Vector3d shift         = drawn.position_metres();
    const double turn                   = drawn.angle_radians();
    const double travel                 = drawn.pitch();
    const rotation turning              = drawn.rotation_member();
    const rigid_motion::screw_ops screw = rigid_motion::baseline().screw;

    seen[10] = seen[10] ||
            answers_apart(held.robot.ik_solve_pose(*composed, shared_motions().frame, tool, start, offset),
                          bent.robot.ik_solve_pose(*composed, shared_motions().frame, tool, start, offset));
    seen[11] = seen[11] || answers_apart(held.robot.ik_solve_flange_pose(*composed, *reached, start), bent.robot.ik_solve_flange_pose(*composed, *reached, start));
    seen[12] = seen[12] || answers_apart(held.motion.task_space_pose(*composed, *reached, start), bent.motion.task_space_pose(*composed, *reached, start));
    seen[13] = seen[13] ||
            answers_apart(held.motion.task_space_screw(screw, *composed, *reached, direction, point, turn, travel, start),
                          bent.motion.task_space_screw(screw, *composed, *reached, direction, point, turn, travel, start));
    seen[14] = seen[14] ||
            answers_apart(held.motion.tool_frame_displace(*composed, *reached, shift, turning, start), bent.motion.tool_frame_displace(*composed, *reached, shift, turning, start));

    const expected<transform, refusal> stepped = composed->fk_solve(manipulator::joint_vector(start + manipulator::joint_vector::Constant(start.size(), waypoint_step_radians)));
    if(!stepped)
        return;

    const std::array<transform, 1> route{*stepped};

    seen[16] = seen[16] ||
            answers_apart(held.trajectory.task_space_waypoints(*composed, route, start, example.chain.limits),
                          bent.trajectory.task_space_waypoints(*composed, route, start, example.chain.limits));
}

std::array<bool, every_row> apart_over_the_run(const manipulator::capabilities &held, const manipulator::capabilities &bent)
{
    std::array<bool, every_row> seen{};

    for(std::size_t index = 0; index < probed_cases; ++index)
    {
        case_source drawn = case_source::at_case(recorded_seed, "manipulator.bend_control", spread::bulk, index);
        note_answers_apart(held, bent, drawn, seen);
    }

    return seen;
}

}

#endif
