#include "iterated_solve.h"

#include "captured_log.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/kinematics.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/optimization/capabilities.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <cmath>
#include <limits>
#include <string>
#include <cstddef>
#include <utility>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

constexpr double upper_arm = 0.5;
constexpr double forearm   = 0.4;

const rigid_motion::screw_ops reference_screw  = rigid_motion::baseline().screw;
const rigid_motion::frame_ops reference_frames = rigid_motion::baseline().frame;

// A planar two-link arm: both joints rotate about the world z, the second sits at the far end of the
// first link, and the tool frame is the far end of the second. Lynch & Park, Modern Robotics,
// example 4.3 -- a screw axis of a revolute joint is (w, -w x q) for a point q on the axis.
screw_chain planar_arm(double bound = 3.0)
{
    screw_axis shoulder;
    shoulder << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    screw_axis elbow;
    elbow << 0.0, 0.0, 1.0, 0.0, -upper_arm, 0.0;

    transform home = transform::Identity();
    home(0, 3)     = upper_arm + forearm;

    joint_limits bounds{};
    bounds.velocity       = joint_vector::Constant(2, 1.0);
    bounds.acceleration   = joint_vector::Constant(2, 2.0);
    bounds.lower_position = joint_vector::Constant(2, -bound);
    bounds.upper_position = joint_vector::Constant(2, bound);

    return screw_chain(home, {shoulder, elbow}, bounds);
}

joint_vector configuration(double first, double second)
{
    joint_vector q(2);
    q << first, second;

    return q;
}

const solver_parameters tight_parameters(1.0e-10, 1.0e-10, 400u);

std::size_t body_forward_calls  = 0;
std::size_t body_jacobian_calls = 0;

expected<transform, refusal> counted_body_forward(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &m,
                                                  std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    ++body_forward_calls;

    return manipulator::body_forward_kinematics(screw, frames, m, space_screws, theta);
}

expected<jacobian, refusal> counted_body_jacobian(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward, const transform &m,
                                                  std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    ++body_jacobian_calls;

    return manipulator::body_jacobian(screw, frames, forward, m, space_screws, theta);
}

expected<jacobian, refusal> refusing_on_second(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward, const transform &m,
                                               std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    if(body_jacobian_calls++ > 0)
        return unexpected(refusal::degenerate);

    return manipulator::body_jacobian(screw, frames, forward, m, space_screws, theta);
}

expected<jacobian, refusal> one_column_short(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const forward_kinematics_ops &, const transform &,
                                             std::span<const screw_axis>, const joint_vector &theta)
{
    return jacobian(jacobian::Zero(6, theta.size() - 1));
}

expected<transform, refusal> refusing_its_input(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const transform &, std::span<const screw_axis>, const joint_vector &)
{
    return unexpected(refusal::unsupported_input);
}

forward_kinematics_ops counted_forward()
{
    forward_kinematics_ops forward  = manipulator::baseline().fk;
    forward.body_forward_kinematics = &counted_body_forward;

    return forward;
}

differential_kinematics_ops differential_with(decltype(differential_kinematics_ops::body_jacobian) body)
{
    differential_kinematics_ops differential = manipulator::baseline().dk;
    differential.body_jacobian               = body;

    return differential;
}

kinematics reference_composition(const screw_chain &chain, const forward_kinematics_ops &forward, const differential_kinematics_ops &differential)
{
    expected<kinematics, refusal> composed =
            kinematics::compose(chain, forward, differential, manipulator::baseline().ik, reference_screw, reference_frames, optimization::baseline().solver);
    REQUIRE(composed);

    return std::move(*composed);
}

transform reached(const joint_vector &q)
{
    const expected<transform, refusal> pose = reference_composition(planar_arm(), manipulator::baseline().fk, manipulator::baseline().dk).fk_solve(q);
    REQUIRE(pose);

    return *pose;
}

expected<void, refusal> solved_under(const differential_kinematics_ops &differential, const screw_chain &chain, const transform &target, const joint_vector &seed, ik_result &answer)
{
    return iterated(reference_screw, reference_frames, manipulator::baseline().fk, differential, chain, target, seed, tight_parameters, answer);
}

refusal refused_before_stepping(const screw_chain &chain, const transform &target, const joint_vector &seed)
{
    ik_result answer;
    const expected<void, refusal> solved = solved_under(differential_with(&counted_body_jacobian), chain, target, seed, answer);
    REQUIRE_FALSE(solved.has_value());
    CHECK(answer.solutions.empty());
    CHECK(answer.iterations.empty());

    return solved.error();
}

struct refused_solve
{
    refusal kind;
    std::size_t iterates;
    std::string said;
};

refused_solve refused_under(const rigid_motion::screw_ops &screw, const forward_kinematics_ops &forward, const differential_kinematics_ops &differential)
{
    const transform target = reached(configuration(0.4, -0.7));
    ik_result answer;
    const tests::captured_log captured;
    const expected<void, refusal> solved = iterated(screw, reference_frames, forward, differential, planar_arm(), target, configuration(0.1, -0.1), tight_parameters, answer);
    REQUIRE_FALSE(solved.has_value());
    CHECK(answer.solutions.empty());

    return refused_solve{solved.error(), answer.iterations.size(), captured.text()};
}

std::size_t occurrences(const std::string &text, const std::string &line)
{
    std::size_t count = 0;
    for(std::size_t at = text.find(line); at != std::string::npos; at = text.find(line, at + line.size()))
        ++count;

    return count;
}

std::size_t times_named(const std::string &said, const std::string &slot)
{
    return occurrences(said, "'" + slot + "' is not implemented, so the inverse kinematics reading it answers no configuration");
}

}

TEST_CASE("a_target_reached_only_outside_the_joint_bounds_is_answered_there_as_the_iteration_converges")
{
    const kinematics solver = reference_composition(planar_arm(1.0), manipulator::baseline().fk, manipulator::baseline().dk);
    const transform target  = reached(configuration(2.0, -1.5));

    const expected<joint_vector, refusal> answer = solver.ik_solve(target, configuration(1.8, -1.3), tight_parameters);

    REQUIRE(answer.has_value());
    CHECK(std::abs((*answer)[0]) > 1.0);
    const expected<transform, refusal> pose = solver.fk_solve(*answer);
    REQUIRE(pose.has_value());
    CHECK(is_approx_equal(*pose, target, 1.0e-6));
}

TEST_CASE("the_reference_reads_its_pose_and_jacobian_through_the_operations_it_is_handed")
{
    const kinematics solver = reference_composition(planar_arm(), counted_forward(), differential_with(&counted_body_jacobian));
    body_forward_calls      = 0;
    body_jacobian_calls     = 0;

    REQUIRE(solver.ik_solve(reached(configuration(0.4, -0.7)), configuration(0.1, -0.1), tight_parameters).has_value());

    CHECK(body_forward_calls > 0);
    CHECK(body_jacobian_calls > 0);
}

TEST_CASE("the_record_opens_with_the_seed_at_index_zero_with_no_step_behind_it")
{
    const kinematics solver = reference_composition(planar_arm(), manipulator::baseline().fk, manipulator::baseline().dk);
    const joint_vector seed = configuration(0.1, -0.1);

    REQUIRE(solver.ik_solve(reached(configuration(0.4, -0.7)), seed, tight_parameters).has_value());

    REQUIRE(solver.iterations().size() > 1u);
    const iteration_state &first = solver.iterations().front();
    CHECK(first.joint_positions == seed);
    CHECK(first.step_norm == 0.0);
    CHECK(first.index == 0u);
    CHECK(first.linear_error > 0.0);
}

TEST_CASE("a_jacobian_of_another_width_ends_the_solve_with_no_solution_before_it_is_used")
{
    ik_result answer;

    const expected<void, refusal> solved = solved_under(differential_with(&one_column_short), planar_arm(), reached(configuration(0.4, -0.7)), configuration(0.1, -0.1), answer);

    REQUIRE_FALSE(solved.has_value());
    CHECK(solved.error() == refusal::no_solution);
    CHECK(answer.solutions.empty());
    REQUIRE(answer.iterations.size() == 1u);
    CHECK(answer.iterations.front().joint_positions == configuration(0.1, -0.1));
}

TEST_CASE("a_jacobian_that_refuses_ends_the_solve_with_no_solution_keeping_the_steps_already_taken")
{
    body_jacobian_calls = 0;
    ik_result answer;

    const expected<void, refusal> solved = solved_under(differential_with(&refusing_on_second), planar_arm(), reached(configuration(0.4, -0.7)), configuration(0.1, -0.1), answer);

    REQUIRE_FALSE(solved.has_value());
    CHECK(solved.error() == refusal::no_solution);
    CHECK(answer.solutions.empty());
    REQUIRE(answer.iterations.size() == 2u);
    CHECK(answer.iterations.back().index == 1u);
}

TEST_CASE("every_entry_refusal_is_answered_before_any_step_is_taken")
{
    body_jacobian_calls    = 0;
    const transform target = reached(configuration(0.4, -0.7));
    transform stretched    = target;
    stretched.block<3, 3>(0, 0) *= 2.0;
    screw_chain off_unit = planar_arm();
    off_unit.space_screws[0] *= 0.5;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    CHECK(refused_before_stepping(planar_arm(), target, joint_vector::Zero(3)) == refusal::unsupported_input);
    CHECK(refused_before_stepping(planar_arm(), stretched, configuration(0.1, -0.1)) == refusal::degenerate);
    CHECK(refused_before_stepping(off_unit, target, configuration(0.1, -0.1)) == refusal::degenerate);
    CHECK(refused_before_stepping(planar_arm(), target, configuration(nan, -0.1)) == refusal::no_solution);
    CHECK(refused_before_stepping(planar_arm(), target, configuration(0.1, inf)) == refusal::no_solution);
    CHECK(body_jacobian_calls == 0u);
}

TEST_CASE("a_forward_map_refusing_not_implemented_at_the_seed_is_answered_as_such_and_named_once_in_the_log")
{
    forward_kinematics_ops forward  = manipulator::baseline().fk;
    forward.body_forward_kinematics = forward_kinematics_ops{}.body_forward_kinematics;

    const refused_solve refused = refused_under(reference_screw, forward, manipulator::baseline().dk);

    CHECK(refused.kind == refusal::not_implemented);
    CHECK(refused.iterates == 0u);
    CHECK(times_named(refused.said, "fk.body_forward_kinematics") == 1u);
}

TEST_CASE("a_jacobian_or_logarithm_refusing_not_implemented_is_answered_as_such_and_named_once_in_the_log")
{
    const refused_solve jacobian_refused = refused_under(reference_screw, manipulator::baseline().fk, differential_with(differential_kinematics_ops{}.body_jacobian));
    CHECK(jacobian_refused.kind == refusal::not_implemented);
    CHECK(jacobian_refused.iterates == 1u);
    CHECK(times_named(jacobian_refused.said, "dk.body_jacobian") == 1u);

    rigid_motion::screw_ops screw         = reference_screw;
    screw.matrix_logarithm_se3            = rigid_motion::screw_ops{}.matrix_logarithm_se3;
    const refused_solve logarithm_refused = refused_under(screw, manipulator::baseline().fk, manipulator::baseline().dk);
    CHECK(logarithm_refused.kind == refusal::not_implemented);
    CHECK(logarithm_refused.iterates == 0u);
    CHECK(times_named(logarithm_refused.said, "screw.matrix_logarithm_se3") == 1u);
}

TEST_CASE("a_slot_refusing_anything_but_not_implemented_is_answered_no_solution_and_names_nothing")
{
    forward_kinematics_ops forward  = manipulator::baseline().fk;
    forward.body_forward_kinematics = &refusing_its_input;

    const refused_solve refused = refused_under(reference_screw, forward, manipulator::baseline().dk);

    CHECK(refused.kind == refusal::no_solution);
    CHECK(refused.said.find("is not implemented") == std::string::npos);
}
