#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_TWO_JOINT_BINDINGS_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_TWO_JOINT_BINDINGS_H

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <functional>

// One two-joint chain and one binding per property under test, so that a case says what it is about
// by which slot it assigns and every other slot stays inert.
namespace praxis::fixture {

using namespace manipulator;

using selector = std::function<std::optional<std::size_t>(std::span<const joint_vector>)>;

inline joint_limits two_joint_bounds()
{
    joint_limits bounds{};
    bounds.velocity       = joint_vector::Constant(2, 1.0);
    bounds.acceleration   = joint_vector::Constant(2, 4.0);
    bounds.lower_position = joint_vector::Constant(2, -1.5);
    bounds.upper_position = joint_vector::Constant(2, 1.5);

    return bounds;
}

inline screw_chain two_joint_chain()
{
    return screw_chain(transform::Identity(), {screw_axis::Zero(), screw_axis::Zero()}, two_joint_bounds());
}

inline expected<transform, refusal> lifting_forward_kinematics(const transform &m, std::span<const screw_axis>, const joint_vector &theta)
{
    transform pose = m;
    pose(2, 3) += theta.size() > 0 ? theta[0] : 0.0;

    return pose;
}

inline expected<jacobian, refusal> counting_space_jacobian(std::span<const screw_axis> space_screws, const joint_vector &)
{
    return jacobian::Constant(6, static_cast<Eigen::Index>(space_screws.size()), 2.0);
}

inline expected<std::vector<screw_axis>, refusal> mirrored_body_screws(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const transform &,
                                                                       std::span<const screw_axis> space_screws)
{
    return std::vector<screw_axis>(space_screws.size(), screw_axis::Constant(3.0));
}

inline expected<std::vector<screw_axis>, refusal> unreadable_body_screws(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const transform &,
                                                                         std::span<const screw_axis>)
{
    return unexpected(refusal::degenerate);
}

// Reads the first screw it is handed, and the derived chain's screws differ from the space chain's,
// so which of the two the holder passed is readable off the answer.
inline expected<transform, refusal> screw_reading_body_forward_kinematics(const rigid_motion::frame_ops &, const transform &m, std::span<const screw_axis> body_screws,
                                                                          const joint_vector &)
{
    transform pose = m;
    pose(2, 3)     = body_screws.empty() ? 0.0 : body_screws.front()[0];

    return pose;
}

// Four different enumerators, so a forwarder that classified a failure of its own instead of carrying
// the slot's would report the wrong one rather than merely reporting.
inline expected<transform, refusal> degenerate_forward_kinematics(const transform &, std::span<const screw_axis>, const joint_vector &)
{
    return unexpected(refusal::degenerate);
}

inline expected<jacobian, refusal> unsupported_space_jacobian(std::span<const screw_axis>, const joint_vector &)
{
    return unexpected(refusal::unsupported_input);
}

inline expected<jacobian, refusal> exhausted_body_jacobian(std::span<const screw_axis>, const joint_vector &)
{
    return unexpected(refusal::no_solution);
}

inline expected<void, refusal> degenerate_inverse_kinematics(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const transform &,
                                                             const joint_vector &, const solver_parameters &, ik_result &)
{
    return unexpected(refusal::degenerate);
}

// Shaped the way a solve written against this seam is: it asks the forward maps it was handed rather
// than computing its own, so what it reads is whatever the composition bound.
inline expected<void, refusal> fk_reading_inverse_kinematics(const forward_kinematics_ops &forward, const differential_kinematics_ops &, const screw_chain &chain, const transform &,
                                                             const joint_vector &j0, const solver_parameters &, ik_result &answer)
{
    const expected<transform, refusal> reached = forward.forward_kinematics(chain.home, chain.space_screws, j0);
    if(!reached)
        return unexpected(reached.error());

    answer.solutions.emplace_back(joint_vector::Constant(j0.size(), (*reached)(2, 3)));
    answer.iterations.push_back(iteration_state{j0, 0.5, 0.125, 0.25, 0u});

    return {};
}

inline expected<void, refusal> two_solution_inverse_kinematics(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const transform &,
                                                               const joint_vector &j0, const solver_parameters &, ik_result &answer)
{
    answer.solutions.emplace_back(joint_vector::Constant(j0.size(), 1.0));
    answer.solutions.emplace_back(joint_vector::Constant(j0.size(), 2.0));

    return {};
}

// Answers one configuration and records nothing, which is the shape of a closed form: what a case
// reads off it is which of the two routes the holder took rather than what any mathematics found.
inline expected<void, refusal> one_answer_analytic_inverse_kinematics(const forward_kinematics_ops &, const screw_chain &chain, const transform &, ik_result &answer)
{
    answer.solutions.emplace_back(joint_vector::Constant(static_cast<Eigen::Index>(chain.joint_count()), 3.0));

    return {};
}

inline expected<void, refusal> converging_on_nothing(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const transform &, const joint_vector &,
                                                     const solver_parameters &, ik_result &)
{
    return {};
}

// Every holder assembled here reads the same chain and binds only what its case is about, which is
// what makes "bound here, inert there" comparable case by case.
inline kinematics holding(forward_kinematics_ops forward, differential_kinematics_ops differential, inverse_kinematics_ops inverse)
{
    expected<kinematics, refusal> composed = kinematics::compose(two_joint_chain(), forward, differential, inverse, rigid_motion::baseline().screw, rigid_motion::baseline().frame);
    REQUIRE(composed);

    return std::move(*composed);
}

}

#endif
