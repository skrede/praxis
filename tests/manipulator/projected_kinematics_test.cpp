#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/solver_adapters.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/projected/solver.h"

#include "praxis/evaluation/tolerance.h"

#include <Eigen/Cholesky>

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <cmath>
#include <limits>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <algorithm>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

constexpr double upper_arm = 0.5;
constexpr double forearm   = 0.4;

const rigid_motion::screw_ops reference_screw  = rigid_motion::baseline().screw;
const rigid_motion::frame_ops reference_frames = rigid_motion::baseline().frame;

constexpr inverse_kinematics_ops route_operations{.inverse_kinematics = &body_inverse_kinematics};
constexpr optimization::solver_ops projected_solve{.solve = &optimization::projected::solve};

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

kinematics composed(const screw_chain &chain, inverse_kinematics_ops inverse, const optimization::solver_ops &solving)
{
    expected<kinematics, refusal> solver =
            manipulator::make_kinematics(chain, manipulator::baseline().fk, manipulator::baseline().dk, inverse, reference_screw, reference_frames, solving);
    REQUIRE(solver);

    return std::move(*solver);
}

kinematics route_solver(const screw_chain &chain)
{
    return composed(chain, route_operations, projected_solve);
}

// For a chain the facade would refuse to hold, the slot is called directly.
expected<void, refusal> solved_through(inverse_kinematics_ops inverse, const optimization::solver_ops &solving, const screw_chain &chain, const transform &target,
                                       const joint_vector &seed, const solver_parameters &parameters, ik_result &answer)
{
    return inverse.inverse_kinematics(reference_screw, reference_frames, manipulator::baseline().fk, manipulator::baseline().dk, solving, chain, target, seed, parameters, answer);
}

expected<void, refusal> routed(const screw_chain &chain, const transform &target, const joint_vector &seed, const solver_parameters &parameters, ik_result &answer)
{
    return solved_through(route_operations, projected_solve, chain, target, seed, parameters, answer);
}

transform pose_of(const screw_chain &chain, const joint_vector &q)
{
    const expected<transform, refusal> pose = manipulator::forward_kinematics(reference_screw, chain.home, chain.space_screws, q);
    REQUIRE(pose);

    return *pose;
}

transform reached(const kinematics &solver, const joint_vector &q)
{
    const expected<transform, refusal> pose = solver.fk_solve(q);
    REQUIRE(pose);

    return *pose;
}

bool inside(const screw_chain &chain, const joint_vector &q)
{
    const Eigen::Index bounded = chain.limits.lower_position.size();

    return (q.head(bounded).array() >= chain.limits.lower_position.array()).all() && (q.head(bounded).array() <= chain.limits.upper_position.array()).all();
}

bool all_inside(const screw_chain &chain, std::span<const iteration_state> states)
{
    return std::ranges::all_of(states, [&](const iteration_state &state) { return inside(chain, state.joint_positions); });
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

struct entry
{
    screw_chain chain;
    transform desired;
    joint_vector seed;
    refusal kind;
};

// A seed of the wrong width, a target that is not a rigid motion, a screw off unit length, a NaN seed,
// and an infinite seed on a sliding joint, which a clamp would make finite.
std::vector<entry> entries_the_book_refuses()
{
    const screw_chain chain = planar_arm();
    const transform target  = pose_of(chain, configuration(0.4, -0.7));
    transform sheared       = target;
    sheared(0, 1)           = 0.5;
    screw_chain blunted     = planar_arm();
    blunted.space_screws[0] << 0.0, 0.0, 0.5, 0.0, 0.0, 0.0;
    screw_chain sliding = planar_arm();
    sliding.space_screws[1] << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;

    return {{chain, target, joint_vector::Constant(3, 0.1), refusal::unsupported_input},
            {chain, sheared, configuration(0.1, -0.1), refusal::degenerate},
            {blunted, target, configuration(0.1, -0.1), refusal::degenerate},
            {chain, target, configuration(std::numeric_limits<double>::quiet_NaN(), -0.1), refusal::no_solution},
            {sliding, pose_of(sliding, configuration(0.4, 0.1)), configuration(0.1, std::numeric_limits<double>::infinity()), refusal::no_solution}};
}

// (J_b^T J_b + damping I)^-1 J_b^T V_b.
joint_vector damped_step(const jacobian &jb, const twist &error, double damping)
{
    return (jb.transpose() * jb + damping * Eigen::MatrixXd::Identity(jb.cols(), jb.cols())).llt().solve(jb.transpose() * error);
}

}

TEST_CASE("the_body_adapter_bound_with_the_projected_solve_reaches_a_target_inside_the_bounds")
{
    const screw_chain chain = planar_arm();
    const kinematics solver = route_solver(chain);
    const transform target  = reached(solver, configuration(0.4, -0.7));

    const expected<joint_vector, refusal> solution = solver.ik_solve(target, configuration(0.1, -0.1), tight_parameters);

    REQUIRE(solution);
    CHECK(is_approx_equal(reached(solver, *solution), target, 1.0e-9));
    CHECK(inside(chain, *solution));
}

TEST_CASE("every_iterate_lies_inside_the_bounds_where_the_book_iterates_leave_them")
{
    const screw_chain chain = planar_arm();
    const kinematics book   = composed(chain, manipulator::baseline().ik, optimization::baseline().solver);
    const kinematics solver = route_solver(chain);
    const transform target  = reached(solver, configuration(0.4, -0.7));
    const joint_vector seed = configuration(-2.9, -2.9);

    REQUIRE(book.ik_solve(target, seed, tight_parameters));
    CHECK_FALSE(all_inside(chain, book.iterations()));
    static_cast<void>(solver.ik_solve(target, seed, tight_parameters));

    const std::span<const iteration_state> states = solver.iterations();
    REQUIRE(states.size() > 1u);
    CHECK(states.front().joint_positions == seed);
    CHECK(all_inside(chain, states));
    for(std::size_t i = 0; i < states.size(); ++i)
    {
        CHECK(states[i].index == static_cast<std::uint32_t>(i));
        CHECK(states[i].step_norm == (i == 0u ? 0.0 : (states[i].joint_positions - states[i - 1u].joint_positions).norm()));
    }
}

// One joint about z bounded [0, 0.5], the target its pose at 1 rad, the seed 0.5: the first projected
// point is the seed itself.
TEST_CASE("a_projected_iterate_equal_to_the_one_before_is_refused_keeping_the_seed")
{
    const screw_chain chain = one_joint_about_z(0.0, 0.5);
    const transform target  = rigid_motion::matrix_exponential_screw(chain.space_screws[0], 1.0) * chain.home;

    ik_result answer;
    const expected<void, refusal> outcome = routed(chain, target, joint_vector::Constant(1, 0.5), solver_parameters(), answer);

    REQUIRE_FALSE(outcome.has_value());
    CHECK(outcome.error() == refusal::no_solution);
    CHECK(answer.solutions.empty());
    REQUIRE(answer.iterations.size() == 1u);
    CHECK(answer.iterations.front().joint_positions == joint_vector::Constant(1, 0.5));
}

TEST_CASE("every_entry_refusal_is_the_adapters_and_the_one_the_book_solve_answers")
{
    for(const entry &one : entries_the_book_refuses())
    {
        ik_result routed_answer;
        ik_result book_answer;
        const expected<void, refusal> route = routed(one.chain, one.desired, one.seed, tight_parameters, routed_answer);
        const expected<void, refusal> book =
                solved_through(manipulator::baseline().ik, optimization::baseline().solver, one.chain, one.desired, one.seed, tight_parameters, book_answer);
        REQUIRE_FALSE(route.has_value());
        REQUIRE_FALSE(book.has_value());
        CHECK(route.error() == one.kind);
        CHECK(book.error() == one.kind);
        CHECK(routed_answer.iterations.empty());
    }
}

TEST_CASE("a_joint_without_a_bound_pair_stays_free")
{
    screw_chain chain           = planar_arm();
    chain.limits.lower_position = joint_vector::Constant(1, -1.0);
    chain.limits.upper_position = joint_vector::Constant(1, 1.0);
    const transform target      = pose_of(chain, configuration(0.4, -2.0));

    ik_result answer;
    REQUIRE(routed(chain, target, configuration(0.1, -0.1), tight_parameters, answer));

    REQUIRE(answer.solutions.size() == 1u);
    CHECK(is_approx_equal(pose_of(chain, answer.solutions.front()), target, 1.0e-9));
    CHECK(std::abs(answer.solutions.front()[1]) > 1.0);
    CHECK(all_inside(chain, answer.iterations));
}

TEST_CASE("the_first_step_is_the_levenberg_marquardt_step_damped_by_the_error_and_the_bias")
{
    const screw_chain chain = planar_arm();
    const kinematics solver = route_solver(chain);
    const joint_vector seed = configuration(0.1, -0.1);
    const transform target  = reached(solver, configuration(0.4, -0.7));
    REQUIRE(solver.ik_solve(target, seed, tight_parameters));

    const expected<jacobian, refusal> jb = manipulator::body_jacobian(reference_screw, reference_frames, manipulator::baseline().fk, chain.home, chain.space_screws, seed);
    const expected<std::pair<screw_axis, double>, refusal> logarithm = rigid_motion::matrix_logarithm_se3(transform(rigid_motion::inverse(reached(solver, seed)) * target));
    REQUIRE(jb);
    REQUIRE(logarithm);

    const twist error   = logarithm->first * logarithm->second;
    const double energy = 0.5 * error.dot(error);

    REQUIRE(solver.iterations().size() > 1u);
    const joint_vector &first = solver.iterations()[1].joint_positions;
    CHECK(is_approx_equal(first, joint_vector(seed + damped_step(*jb, error, energy + 1.0e-8)), 1.0e-12));
    CHECK((first - (seed + damped_step(*jb, error, 1.0e-8))).norm() > 1.0e-9);
}

TEST_CASE("an_unreachable_target_is_refused_with_every_iterate_inside_the_bounds")
{
    screw_chain chain              = planar_arm();
    chain.limits.lower_position[1] = 0.1;
    transform far_away             = transform::Identity();
    far_away(0, 3)                 = 10.0 * (upper_arm + forearm);
    const solver_parameters parameters;

    ik_result answer;
    const expected<void, refusal> outcome = routed(chain, far_away, configuration(0.2, 0.2), parameters, answer);

    REQUIRE_FALSE(outcome.has_value());
    CHECK(outcome.error() == refusal::no_solution);
    CHECK(answer.solutions.empty());
    CHECK(answer.iterations.size() <= parameters.max_iterations_per_attempt + 1u);
    CHECK(all_inside(chain, answer.iterations));
}

TEST_CASE("a_budget_of_zero_judges_the_seed_and_takes_no_step")
{
    const kinematics solver = route_solver(planar_arm());
    const joint_vector seed = configuration(0.1, -0.1);

    const expected<joint_vector, refusal> solution = solver.ik_solve(reached(solver, configuration(0.4, -0.7)), seed, solver_parameters(1.0e-7, 1.0e-7, 0u));

    REQUIRE_FALSE(solution.has_value());
    CHECK(solution.error() == refusal::no_solution);
    REQUIRE(solver.iterations().size() == 1u);
    CHECK(solver.iterations().front().joint_positions == seed);
}
