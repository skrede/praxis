#include "recorded_chain.h"
#include "three_link_arm.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/rigid_motion/baseline/screw.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <vector>
#include <cstddef>
#include <utility>
#include <cmath>
#include <algorithm>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

// The two halves of the body twist between the pose a configuration reaches and the pose asked for:
// V_b = log(T_sb^-1 T_sd), angular part leading. Lynch & Park, Modern Robotics, eq. (6.4). Taken with
// this tree's own logarithm rather than the dependency's, so a value recorded by the solve and the
// value checked against it do not come from the same computation.
std::pair<double, double> body_halves(const kinematics &solver, const transform &target, const joint_vector &q)
{
    const auto [axis, angle] = rigid_motion::matrix_logarithm_se3(solver.fk_solve(q).value().inverse() * target).value();
    const twist body_error   = axis * angle;

    return {body_error.head<3>().norm(), body_error.tail<3>().norm()};
}

// Away from the outstretched and folded configurations of the planar arm, and away from the wrist
// singularity the six-joint fixture carries at its zero configuration.
std::vector<joint_vector> planar_grid()
{
    std::vector<joint_vector> grid;
    for(double first : {-1.0, -0.3, 0.4, 1.1})
        for(double second : {-1.2, -0.5, 0.6, 1.3})
            for(double third : {-0.9, -0.2, 0.7, 1.4})
                grid.push_back(fixture::arm_configuration(first, second, third));

    return grid;
}

std::vector<joint_vector> recorded_grid()
{
    std::vector<joint_vector> grid;
    for(double shoulder : {-0.6, 0.5})
        for(double lift : {-1.1, -0.4})
            for(double elbow : {0.3, 1.0})
                for(double roll : {-0.7, 0.8})
                    grid.push_back(fixture::recorded_configuration(shoulder, lift, elbow, roll, 0.6, -0.5));

    return grid;
}

struct round_trips
{
    std::size_t converged;
    std::size_t attempted;
    double worst_angular;
    double worst_linear;
    std::size_t worst_work;
};

// The pose a configuration reaches is handed back as the target from a seed a fixed distance away,
// and what comes back is measured against the pose rather than against the configuration -- an arm
// has more than one configuration for a pose.
round_trips round_trip(const screw_chain &chain, const std::vector<joint_vector> &grid, const solver_parameters &parameters)
{
    const kinematics solver = manipulator::make_kinematics(chain, manipulator::baseline().fk, manipulator::baseline().dk, manipulator::baseline().ik, rigid_motion::baseline().screw,
                                                           rigid_motion::baseline().frame)
                                      .value();
    round_trips seen{0u, grid.size(), 0.0, 0.0, 0u};

    for(const joint_vector &q : grid)
    {
        const transform target                         = solver.fk_solve(q).value();
        const joint_vector seed                        = q + joint_vector::Constant(q.size(), 0.3);
        const expected<joint_vector, refusal> solution = solver.ik_solve(target, seed, parameters);

        seen.worst_work = std::max(seen.worst_work, solver.iterations().size());
        if(!solution)
            continue;

        const auto [angular, linear] = body_halves(solver, target, *solution);
        if(angular >= 1.0e-3 || linear >= 1.0e-3)
            continue;

        ++seen.converged;
        seen.worst_angular = std::max(seen.worst_angular, angular);
        seen.worst_linear  = std::max(seen.worst_linear, linear);
    }

    return seen;
}

// Twenty times the worst residual either fixture produced at the defaults, so the bound survives a
// toolchain that rounds differently rather than sitting on the number this machine measured.
constexpr double asserted_residual = 1.0e-6;

}

TEST_CASE("the_default_parameters_carry_every_sampled_configuration_through_a_round_trip")
{
    const solver_parameters defaults;
    const round_trips planar   = round_trip(fixture::three_link_arm(), planar_grid(), defaults);
    const round_trips recorded = round_trip(fixture::recorded_arm(), recorded_grid(), defaults);

    CHECK(planar.converged == planar.attempted);
    CHECK(recorded.converged == recorded.attempted);
    CHECK(planar.worst_angular < asserted_residual);
    CHECK(planar.worst_linear < asserted_residual);
    CHECK(recorded.worst_angular < asserted_residual);
    CHECK(recorded.worst_linear < asserted_residual);

    // The budget is not what stops these solves; it is there for the ones it does stop.
    CHECK(planar.worst_work * 4u < defaults.max_iterations_per_attempt);
    CHECK(recorded.worst_work * 4u < defaults.max_iterations_per_attempt);
}

TEST_CASE("the_position_tolerance_is_the_tightest_at_which_both_fixtures_still_converge")
{
    const solver_parameters defaults;
    const solver_parameters tighter(defaults.position_tol / 10.0, defaults.orientation_tol, defaults.max_iterations_per_attempt);
    const solver_parameters looser(defaults.position_tol * 10.0, defaults.orientation_tol, defaults.max_iterations_per_attempt);

    // An order tighter and the damped least squares stalls on configurations it otherwise carries.
    const round_trips stalled = round_trip(fixture::three_link_arm(), planar_grid(), tighter);
    CHECK(stalled.converged < stalled.attempted);

    // An order looser and every configuration still arrives, but visibly further from the target.
    const round_trips slack  = round_trip(fixture::three_link_arm(), planar_grid(), looser);
    const round_trips chosen = round_trip(fixture::three_link_arm(), planar_grid(), defaults);
    CHECK(slack.converged == slack.attempted);
    CHECK(slack.worst_linear > chosen.worst_linear * 10.0);
}

TEST_CASE("the_iteration_budget_is_what_decides_whether_a_solve_finishes")
{
    const solver_parameters defaults;
    const solver_parameters starved(defaults.position_tol, defaults.orientation_tol, 6u);
    const solver_parameters sufficient(defaults.position_tol, defaults.orientation_tol, 7u);

    const round_trips cut  = round_trip(fixture::three_link_arm(), planar_grid(), starved);
    const round_trips just = round_trip(fixture::three_link_arm(), planar_grid(), sufficient);

    CHECK(cut.converged < cut.attempted);
    CHECK(just.converged == just.attempted);
}

// A count above the range of the signed type the dependency takes is clamped to that type's maximum.
// Unclamped it wraps negative, and a negative limit is reached by the first iteration, so a request
// for more effort than the type can hold becomes a request for none.
TEST_CASE("a_budget_beyond_the_dependencys_signed_range_still_runs_the_solve")
{
    const solver_parameters defaults;
    const solver_parameters beyond(defaults.position_tol, defaults.orientation_tol, 3'000'000'000u);

    const kinematics solver     = manipulator::make_kinematics(fixture::three_link_arm(), manipulator::baseline().fk, manipulator::baseline().dk, manipulator::baseline().ik,
                                                               rigid_motion::baseline().screw, rigid_motion::baseline().frame)
                                          .value();
    const transform target      = solver.fk_solve(fixture::posed_arm()).value();
    const joint_vector seed     = fixture::arm_configuration(0.1, -0.9, 0.2);
    const joint_vector solution = solver.ik_solve(target, seed, beyond).value();

    const auto [angular, linear] = body_halves(solver, target, solution);
    CHECK(solver.iterations().size() > 1u);
    CHECK(angular < asserted_residual);
    CHECK(linear < asserted_residual);
}

// A target that is neither a pure translation nor a pure rotation, reached on the six-joint fixture
// where the tool can be turned about all three axes: both halves of the body twist stay large and of
// the same order, so neither is near zero and neither carries the whole error. A record that stored
// one scalar in both places would disagree with both of them.
TEST_CASE("the_recorded_errors_are_the_two_halves_of_the_body_twist")
{
    const kinematics solver = manipulator::make_kinematics(fixture::recorded_arm(), manipulator::baseline().fk, manipulator::baseline().dk, manipulator::baseline().ik,
                                                           rigid_motion::baseline().screw, rigid_motion::baseline().frame)
                                      .value();
    const joint_vector seed = fixture::recorded_configuration(-0.4, -0.3, 0.2, 0.9, -0.8, 1.2);
    const transform target  = solver.fk_solve(fixture::recorded_configuration(0.5, -1.1, 1.0, -0.7, 0.6, -0.5)).value();
    REQUIRE(solver.ik_solve(target, seed, solver_parameters()).has_value());
    const std::span<const iteration_state> states = solver.iterations();
    REQUIRE(states.size() > 1u);

    for(const iteration_state &state : states)
    {
        const auto [angular, linear] = body_halves(solver, target, state.joint_positions);
        CHECK(is_approx_equal(state.angular_error, angular, 1.0e-9));
        CHECK(is_approx_equal(state.linear_error, linear, 1.0e-9));
    }

    // Both halves are a material fraction of the whole twist, and they differ from each other, which
    // is what a collapsed split cannot reproduce.
    const iteration_state &first = states.front();
    const double whole           = std::hypot(first.angular_error, first.linear_error);
    CHECK(first.angular_error > 0.3 * whole);
    CHECK(first.linear_error > 0.3 * whole);
    CHECK(first.angular_error > 2.0 * first.linear_error);
}
