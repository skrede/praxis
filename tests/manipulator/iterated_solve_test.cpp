#include "iterated_solve.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/kinematics.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <cstddef>

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
    expected<kinematics, refusal> solver =
            manipulator::make_kinematics(planar_arm(), manipulator::baseline().fk, manipulator::baseline().dk, manipulator::baseline().ik, reference_screw, reference_frames);
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
