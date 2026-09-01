#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_BENT_CHAIN_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_BENT_CHAIN_H

#include "praxis/manipulator/baseline/modeling.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include <meios/model.h>

#include <span>
#include <vector>
#include <cstddef>
#include <utility>

// Bindings for the three slots whose return types no shipped residual folds over. Each answers the
// reference wrong in one stated way, or declines, so that a comparator written for such a shape can
// be held to its own contract rather than only to a number.
namespace praxis::fixture {

using namespace manipulator;

inline std::vector<screw_axis> negated(std::span<const screw_axis> axes)
{
    std::vector<screw_axis> flipped;
    flipped.reserve(axes.size());
    for(const screw_axis &axis : axes)
        flipped.emplace_back(-axis);

    return flipped;
}

inline expected<std::vector<screw_axis>, refusal> every_axis_negated(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &m,
                                                                     std::span<const screw_axis> space_screws)
{
    const expected<std::vector<screw_axis>, refusal> derived = body_screws_from_space(screw, frames, m, space_screws);
    if(!derived)
        return derived;

    return negated(*derived);
}

inline expected<std::vector<screw_axis>, refusal> one_axis_short(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &m,
                                                                 std::span<const screw_axis> space_screws)
{
    expected<std::vector<screw_axis>, refusal> derived = body_screws_from_space(screw, frames, m, space_screws);
    if(!derived)
        return derived;

    derived->pop_back();

    return derived;
}

inline expected<void, refusal> never_solves(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const transform &, const joint_vector &,
                                            const solver_parameters &, ik_result &)
{
    return unexpected(refusal::no_solution);
}

inline expected<void, refusal> never_solves_for_another_reason(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const transform &,
                                                               const joint_vector &, const solver_parameters &, ik_result &)
{
    return unexpected(refusal::unsupported_input);
}

inline expected<void, refusal> answers_without_naming_a_configuration(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const transform &,
                                                                      const joint_vector &, const solver_parameters &, ik_result &)
{
    return {};
}

inline expected<void, refusal> always_answers_the_seed(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const transform &,
                                                       const joint_vector &j0, const solver_parameters &, ik_result &answer)
{
    answer.solutions.push_back(j0);

    return {};
}

inline std::size_t entries = 0;

// A solve reaching the forward map and the Jacobian through the aggregates it is handed, which is what
// a real search does. The comparator asks each side's slot once per drawn case, so this count stands
// at one per drawn case.
inline expected<void, refusal> counts_its_entries(const forward_kinematics_ops &forward, const differential_kinematics_ops &differential, const screw_chain &chain,
                                                  const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer)
{
    ++entries;
    static_cast<void>(forward.forward_kinematics(chain.home, chain.space_screws, j0));
    static_cast<void>(differential.space_jacobian(chain.space_screws, j0));

    return inverse_kinematics(forward, differential, chain, desired, j0, parameters, answer);
}

inline inverse_kinematics_ops solving(decltype(inverse_kinematics_ops::inverse_kinematics) bound)
{
    return inverse_kinematics_ops{bound};
}

inline joint_vector agreeing_configuration;

inline expected<screw_chain, refusal> last_screw_negated(const meios::model<> &model)
{
    const expected<screw_chain, refusal> derived = build_chain(model);
    if(!derived)
        return derived;

    std::vector<screw_axis> screws = derived->space_screws;
    screws.back()                  = screw_axis(-screws.back());

    return screw_chain(derived->home, std::move(screws), derived->limits);
}

inline expected<screw_chain, refusal> one_joint_short(const meios::model<> &model)
{
    expected<screw_chain, refusal> derived = build_chain(model);
    if(!derived)
        return derived;

    derived->space_screws.pop_back();

    return derived;
}

// The wrong screws of `last_screw_negated`, carried onto a home pose that makes the forward map agree
// with the reference at `agreeing_configuration` exactly. FK(M, S, t) is P(S, t) M with P the product
// of the exponentials, so M' = P(S', t)^-1 P(S, t) M answers the reference's own pose at that one t.
inline expected<screw_chain, refusal> agreeing_at_one_configuration(const meios::model<> &model)
{
    const expected<screw_chain, refusal> wrong = last_screw_negated(model);
    const expected<screw_chain, refusal> right = build_chain(model);
    if(!wrong || !right)
        return wrong;

    const transform held = forward_kinematics(transform::Identity(), wrong->space_screws, agreeing_configuration).value();
    const transform ref  = forward_kinematics(transform::Identity(), right->space_screws, agreeing_configuration).value();

    return screw_chain(transform(held.inverse() * ref * right->home), wrong->space_screws, wrong->limits);
}

}

#endif
