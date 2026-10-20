#include "joint_bounds.h"
#include "iterated_solve.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/kinematics.h"
#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/optimization/capabilities.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <limits>
#include <cstddef>
#include <optional>

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
screw_chain planar_arm()
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
    bounds.lower_position = joint_vector::Constant(2, -3.0);
    bounds.upper_position = joint_vector::Constant(2, 3.0);

    return screw_chain(home, {shoulder, elbow}, bounds);
}

joint_vector configuration(double first, double second)
{
    joint_vector q(2);
    q << first, second;

    return q;
}

const solver_parameters tight_parameters(1.0e-10, 1.0e-10, 400u);

transform reached(const joint_vector &q)
{
    expected<kinematics, refusal> solver = manipulator::make_kinematics(planar_arm(), manipulator::baseline().fk, manipulator::baseline().dk, manipulator::baseline().ik,
                                                                        reference_screw, reference_frames, optimization::baseline().solver);
    REQUIRE(solver);
    const expected<transform, refusal> pose = solver->fk_solve(q);
    REQUIRE(pose);

    return *pose;
}

std::size_t steps_taken = 0;

expected<joint_vector, refusal> one_short(const body_target &, const joint_vector &theta, const jacobian &, const twist &)
{
    return joint_vector(theta.head(theta.size() - 1));
}

expected<joint_vector, refusal> refusing_on_second(const body_target &, const joint_vector &theta, const jacobian &, const twist &)
{
    if(steps_taken++ > 0)
        return unexpected(refusal::degenerate);

    return joint_vector(theta + configuration(0.01, 0.01));
}

expected<joint_vector, refusal> counted(const body_target &, const joint_vector &theta, const jacobian &, const twist &)
{
    ++steps_taken;

    return theta;
}

refusal refused_before_stepping(const screw_chain &chain, const transform &target, const joint_vector &seed)
{
    ik_result answer;
    const expected<void, refusal> solved = iterated(chain, target, seed, tight_parameters, &counted, answer);
    REQUIRE_FALSE(solved.has_value());
    CHECK(answer.solutions.empty());
    CHECK(answer.iterations.empty());

    return solved.error();
}

screw_chain one_joint_about_z(double lower, double upper)
{
    screw_axis about_z;
    about_z << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    joint_limits bounds{};
    bounds.lower_position = joint_vector::Constant(1, lower);
    bounds.upper_position = joint_vector::Constant(1, upper);
    transform home        = transform::Identity();
    home(0, 3)            = 0.5;

    return screw_chain(home, {about_z}, bounds);
}

transform pose_of(const screw_chain &chain, const joint_vector &q)
{
    const expected<transform, refusal> pose = manipulator::forward_kinematics(reference_screw, chain.home, chain.space_screws, q);
    REQUIRE(pose);

    return *pose;
}

constexpr double unbounded = std::numeric_limits<double>::infinity();

// {lower, upper, value, named} in degrees.
constexpr std::array<std::array<double, 4>, 18> whole_turns_past_a_bound{{{-120.0, 120.0, 240.0, -120.0},
                                                                          {-120.0, 120.0, 840.0, 120.0},
                                                                          {-120.0, 120.0, -840.0, -120.0},
                                                                          {-135.0, 135.0, 495.0, 135.0},
                                                                          {-135.0, 135.0, -495.0, -135.0},
                                                                          {-99.0, 99.0, 459.0, 99.0},
                                                                          {-99.0, 99.0, -459.0, -99.0},
                                                                          {-93.0, 93.0, 1173.0, 93.0},
                                                                          {-93.0, 93.0, -1173.0, -93.0},
                                                                          {-100.0, 150.0, 510.0, 150.0},
                                                                          {-100.0, 150.0, -460.0, -100.0},
                                                                          {-183.0, 183.0, 1983.0, 183.0},
                                                                          {-183.0, 183.0, -1983.0, -183.0},
                                                                          {-unbounded, 135.0, 495.0, 135.0},
                                                                          {-135.0, unbounded, -495.0, -135.0},
                                                                          {-unbounded, 120.0, 840.0, 120.0},
                                                                          {-120.0, unbounded, -840.0, -120.0},
                                                                          {-unbounded, unbounded, 495.0, 495.0}}};

}

TEST_CASE("a_step_answering_a_configuration_of_another_width_is_refused_before_it_is_used")
{
    ik_result answer;

    const expected<void, refusal> solved = iterated(planar_arm(), reached(configuration(0.4, -0.7)), configuration(0.1, -0.1), tight_parameters, &one_short, answer);

    REQUIRE_FALSE(solved.has_value());
    CHECK(solved.error() == refusal::no_solution);
    CHECK(answer.solutions.empty());
    CHECK(answer.iterations.empty());
}

TEST_CASE("a_step_that_refuses_ends_the_solve_with_no_solution_keeping_the_steps_already_taken")
{
    steps_taken = 0;
    ik_result answer;

    const expected<void, refusal> solved = iterated(planar_arm(), reached(configuration(0.4, -0.7)), configuration(0.1, -0.1), tight_parameters, &refusing_on_second, answer);

    REQUIRE_FALSE(solved.has_value());
    CHECK(solved.error() == refusal::no_solution);
    CHECK(answer.solutions.empty());
    REQUIRE(answer.iterations.size() == 1);
    CHECK(answer.iterations.front().joint_positions.isApprox(configuration(0.11, -0.09)));
}

TEST_CASE("every_entry_refusal_is_answered_before_any_step_is_taken")
{
    steps_taken            = 0;
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
    CHECK(steps_taken == 0);
}

TEST_CASE("seeds_a_whole_number_of_turns_past_a_bound_are_named_inside_the_bounds_and_answered_there_by_the_book_solve")
{
    for(const auto &[lower, upper, value, named] : whole_turns_past_a_bound)
    {
        CAPTURE(lower, upper, value, named);
        const screw_chain chain                      = one_joint_about_z(lower * radians_per_degree, upper * radians_per_degree);
        const joint_vector seed                      = joint_vector::Constant(1, value * radians_per_degree);
        const std::optional<joint_vector> named_seed = named_inside_bounds(chain, seed);
        REQUIRE(named_seed.has_value());
        CHECK(is_approx_equal((*named_seed)[0], named * radians_per_degree, 1.0e-12));
        CHECK(named_inside_bounds(chain, *named_seed) == named_seed);

        ik_result answer;
        CHECK(manipulator::baseline().ik.inverse_kinematics(reference_screw, reference_frames, manipulator::baseline().fk, manipulator::baseline().dk, optimization::baseline().solver,
                                                            chain, pose_of(chain, seed), seed, tight_parameters, answer));
        REQUIRE(answer.solutions.size() == 1u);
        CHECK(is_approx_equal(answer.solutions.front()[0], named * radians_per_degree, 1.0e-12));
        CHECK(is_approx_equal(pose_of(chain, answer.solutions.front()), pose_of(chain, seed), 1.0e-9));
    }
}
