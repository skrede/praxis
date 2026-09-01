#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/evaluation/comparators.h"

#include "praxis/rigid_motion/capabilities.h"

#include <span>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// How many configurations one chain comparison is taken over: a chain of N joints carries 6 + 6N free
// numbers against six constraints each, so this over-determines every width the source draws (N <= 8).
constexpr std::size_t configurations_per_chain = 10u;

// One screw implementation and one frame implementation serve both sides of every row that reaches
// them, so a row measures its own slot rather than a capability neither side is under test for.
const rigid_motion::screw_ops &shared_screw()
{
    static const rigid_motion::screw_ops screw = rigid_motion::baseline().screw;

    return screw;
}

const rigid_motion::frame_ops &shared_frames()
{
    static const rigid_motion::frame_ops frames = rigid_motion::baseline().frame;

    return frames;
}

const modeling_ops &modeling_of(const void *value)
{
    return *static_cast<const modeling_ops *>(value);
}

const forward_kinematics_ops &forward_kinematics_of(const void *value)
{
    return *static_cast<const forward_kinematics_ops *>(value);
}

const inverse_kinematics_ops &inverse_kinematics_of(const void *value)
{
    return *static_cast<const inverse_kinematics_ops *>(value);
}

// Two sequences of screw axes of the same length, read as the six-by-N matrices a fixed linear map
// produces, so the whole sequence is one comparison and its ordering is part of what is compared.
evaluation::residual element_wise_over(std::span<const screw_axis> held, std::span<const screw_axis> against)
{
    const auto taken = static_cast<Eigen::Index>(held.size());
    if(taken == 0)
        return evaluation::residual{evaluation::residual_kind::element_wise, 0.0, 0.0};

    Eigen::MatrixXd here(6, taken);
    Eigen::MatrixXd there(6, taken);
    for(Eigen::Index axis = 0; axis < taken; ++axis)
    {
        here.col(axis)  = held[static_cast<std::size_t>(axis)];
        there.col(axis) = against[static_cast<std::size_t>(axis)];
    }

    return evaluation::element_wise_residual(here, there);
}

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

std::optional<solve_request> drawn_request(evaluation::case_source &drawn, const evaluation_case &example)
{
    const joint_vector seed                = drawn_joints(drawn, example.chain.joint_count());
    const capabilities reference           = baseline();
    expected<kinematics, refusal> composed = kinematics::compose(example.chain, reference.fk, reference.dk, reference.ik, shared_screw(), shared_frames());
    if(!composed)
        return std::nullopt;

    const expected<transform, refusal> reached = composed->fk_solve(example.joints);
    if(!reached)
        return std::nullopt;

    return solve_request{std::move(*composed), reference.fk, reference.dk, *reached, seed};
}

expected<void, refusal> solved_by(const inverse_kinematics_ops &bound, const screw_chain &chain, const solve_request &asked, ik_result &answer)
{
    return bound.inverse_kinematics(asked.forward, asked.differential, chain, asked.target, asked.seed, solver_parameters(), answer);
}

// A pose a chain can reach is reached by more than one configuration, so an answer is read back
// through one shared forward map before it is held against anything.
std::optional<transform> reached_by(const kinematics &reference, const ik_result &answer)
{
    const expected<transform, refusal> pose = reference.fk_solve(answer.solutions.front());
    if(!pose)
        return std::nullopt;

    return *pose;
}

evaluation::case_result agreeing_everywhere(const screw_chain &held, const screw_chain &against, std::span<const joint_vector> asked_at, const evaluation::tolerance_pair &allowed)
{
    evaluation::residual worst{evaluation::residual_kind::pose, 0.0, 0.0};

    for(const joint_vector &theta : asked_at)
    {
        const expected<transform, refusal> here  = forward_kinematics(held.home, held.space_screws, theta);
        const expected<transform, refusal> there = forward_kinematics(against.home, against.space_screws, theta);
        if(!here || !there)
            return unusable(evaluation::residual_kind::pose);

        const evaluation::residual seen = evaluation::pose_residual(*here, *there);
        worst.magnitude                 = std::max(worst.magnitude, seen.magnitude);
        worst.linear_error_metres       = std::max(worst.linear_error_metres, seen.linear_error_metres);
    }

    return judged(worst, allowed);
}

}

evaluation::case_result compare_body_screws_from_space(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const evaluation_case example = drawn_case(drawn);
    const expected<std::vector<screw_axis>, refusal> held =
            forward_kinematics_of(first).body_screws_from_space(shared_screw(), shared_frames(), example.chain.home, example.chain.space_screws);
    const expected<std::vector<screw_axis>, refusal> other =
            forward_kinematics_of(second).body_screws_from_space(shared_screw(), shared_frames(), example.chain.home, example.chain.space_screws);
    if(const std::optional<evaluation::case_result> refused = refusal_outcome(held, other))
        return *refused;

    if(held->size() != other->size())
        return categorically_differed(evaluation::residual_kind::element_wise);

    return judged(element_wise_over(*held, *other), allowed);
}

evaluation::case_result compare_inverse_kinematics(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const evaluation_case example            = drawn_case(drawn);
    const std::optional<solve_request> asked = drawn_request(drawn, example);
    if(!asked)
        return unusable(evaluation::residual_kind::pose);

    ik_result here;
    ik_result there;
    const expected<void, refusal> held  = solved_by(inverse_kinematics_of(first), example.chain, *asked, here);
    const expected<void, refusal> other = solved_by(inverse_kinematics_of(second), example.chain, *asked, there);
    if(const std::optional<evaluation::case_result> refused = refusal_outcome(held, other))
        return *refused;

    if(here.solutions.empty() || there.solutions.empty())
        return categorically_differed(evaluation::residual_kind::pose);

    return reaching_what_was_asked(reached_by(asked->reference, here), reached_by(asked->reference, there), asked->target, allowed);
}

evaluation::case_result compare_build_chain(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const meios::model<> machine = drawn_model(drawn);
    std::vector<joint_vector> asked_at;
    asked_at.reserve(configurations_per_chain);
    for(std::size_t index = 0; index < configurations_per_chain; ++index)
        asked_at.push_back(drawn_joints(drawn, machine.joints.size()));

    const expected<screw_chain, refusal> held  = modeling_of(first).build_chain(machine);
    const expected<screw_chain, refusal> other = modeling_of(second).build_chain(machine);
    if(const std::optional<evaluation::case_result> refused = refusal_outcome(held, other))
        return *refused;
    if(held->joint_count() != other->joint_count())
        return categorically_differed(evaluation::residual_kind::pose);

    return agreeing_everywhere(*held, *other, asked_at, allowed);
}

}
