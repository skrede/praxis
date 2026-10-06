#include "joint_bounds.h"

#include "projected/step_constants.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/kinematics.h"
#include "praxis/manipulator/projected/kinematics.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/evaluation/tolerance.h"

#include <Eigen/Cholesky>

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
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

constexpr inverse_kinematics_ops projected_operations{.inverse_kinematics = &projected::inverse_kinematics};

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

kinematics composed(const screw_chain &chain, inverse_kinematics_ops inverse)
{
    expected<kinematics, refusal> solver = manipulator::make_kinematics(chain, manipulator::baseline().fk, manipulator::baseline().dk, inverse, reference_screw, reference_frames);
    REQUIRE(solver);

    return std::move(*solver);
}

kinematics projected_solver(const screw_chain &chain)
{
    return composed(chain, projected_operations);
}

// For a chain the facade would refuse to hold, the slot is called directly.
expected<void, refusal> solved_through(inverse_kinematics_ops inverse, const screw_chain &chain, const transform &target, const joint_vector &seed, const solver_parameters &parameters,
                                       ik_result &answer)
{
    return inverse.inverse_kinematics(reference_screw, manipulator::baseline().fk, manipulator::baseline().dk, chain, target, seed, parameters, answer);
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

void both_solves_answer_at(const screw_chain &chain, const joint_vector &seed, double named)
{
    for(const inverse_kinematics_ops inverse : {projected_operations, manipulator::baseline().ik})
    {
        ik_result answer;
        CHECK(solved_through(inverse, chain, pose_of(chain, seed), seed, tight_parameters, answer));
        CHECK(answer.solutions.size() == 1u);
        for(const joint_vector &q : answer.solutions)
        {
            CHECK(inside(chain, q));
            CHECK(is_approx_equal(q[0], named, 1.0e-12));
            CHECK(is_approx_equal(pose_of(chain, q), pose_of(chain, seed), 1.0e-9));
        }
    }
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

// Joint 1 turns whole about z, bounded [-170, 170] degrees; joint 2 slides along x, bounded [-0.2, 0.3];
// joint 3 turns whole about z and carries no bound pair.
screw_chain projection_chain()
{
    screw_axis turning;
    turning << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    screw_axis sliding;
    sliding << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;

    joint_limits bounds{};
    bounds.lower_position = joint_vector(2);
    bounds.upper_position = joint_vector(2);
    bounds.lower_position << -170.0 * radians_per_degree, -0.2;
    bounds.upper_position << 170.0 * radians_per_degree, 0.3;

    return screw_chain(transform::Identity(), {turning, sliding, turning}, bounds);
}

joint_vector projected_joints(double first, double second, double third)
{
    joint_vector raw(3);
    raw << first, second, third;

    return projected_inside_bounds(projection_chain(), raw);
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

Eigen::Matrix<double, 6, 1> weights()
{
    Eigen::Matrix<double, 6, 1> diagonal;
    diagonal << projected::rotation_weight, projected::rotation_weight, projected::rotation_weight, projected::translation_weight, projected::translation_weight,
            projected::translation_weight;

    return diagonal;
}

// (J_b^T W_E J_b + damping I)^-1 J_b^T W_E V_b.
joint_vector damped_step(const jacobian &jb, const twist &error, double damping)
{
    const Eigen::MatrixXd normal = jb.transpose() * weights().asDiagonal() * jb;

    return (normal + damping * Eigen::MatrixXd::Identity(jb.cols(), jb.cols())).llt().solve(jb.transpose() * (weights().asDiagonal() * error));
}
}

TEST_CASE("the_projected_solve_binds_by_address_of_and_reaches_a_target_inside_the_bounds")
{
    const screw_chain chain = planar_arm();
    const kinematics solver = projected_solver(chain);
    const transform target  = reached(solver, configuration(0.4, -0.7));

    const expected<joint_vector, refusal> solution = solver.ik_solve(target, configuration(0.1, -0.1), tight_parameters);

    REQUIRE(solution);
    CHECK(is_approx_equal(reached(solver, *solution), target, 1.0e-9));
    CHECK(inside(chain, *solution));
}

TEST_CASE("every_iterate_lies_inside_the_bounds_where_the_book_iterates_leave_them")
{
    const screw_chain chain = planar_arm();
    const kinematics book   = composed(chain, manipulator::baseline().ik);
    const kinematics solver = projected_solver(chain);
    const transform target  = reached(solver, configuration(0.4, -0.7));
    const joint_vector seed = configuration(-2.9, -2.9);

    REQUIRE(book.ik_solve(target, seed, tight_parameters));
    CHECK_FALSE(all_inside(chain, book.iterations()));
    REQUIRE(solver.ik_solve(target, seed, tight_parameters));

    const std::span<const iteration_state> states = solver.iterations();
    REQUIRE_FALSE(states.empty());
    CHECK(all_inside(chain, states));
    for(std::size_t i = 0; i < states.size(); ++i)
    {
        const joint_vector &before = i == 0u ? seed : states[i - 1u].joint_positions;
        CHECK(states[i].index == static_cast<std::uint32_t>(i));
        CHECK(states[i].step_norm == (states[i].joint_positions - before).norm());
    }
}

TEST_CASE("a_seed_outside_the_bounds_is_projected_before_the_first_step_and_not_recorded")
{
    screw_chain chain              = planar_arm();
    chain.limits.lower_position[0] = -1.0;
    chain.limits.upper_position[0] = 1.0;
    const kinematics solver        = projected_solver(chain);
    const joint_vector handed      = configuration(1.5, -0.1);
    const joint_vector projected   = configuration(1.0, -0.1);

    REQUIRE(solver.ik_solve(reached(solver, configuration(0.4, -0.7)), handed, tight_parameters));

    REQUIRE_FALSE(solver.iterations().empty());
    const iteration_state &first = solver.iterations().front();
    CHECK(first.index == 0u);
    CHECK(first.joint_positions != projected);
    CHECK(first.step_norm == (first.joint_positions - projected).norm());
    CHECK(first.step_norm != (first.joint_positions - handed).norm());
}

// One joint about z bounded [0, 0.5], the target its pose at 1 rad, the seed 0.5: the first projected
// point is the seed itself.
TEST_CASE("a_projected_iterate_equal_to_the_one_before_is_refused_with_nothing_recorded")
{
    const screw_chain chain = one_joint_about_z(0.0, 0.5);
    const transform target  = rigid_motion::matrix_exponential_screw(chain.space_screws[0], 1.0) * chain.home;

    ik_result answer;
    const expected<void, refusal> outcome = solved_through(projected_operations, chain, target, joint_vector::Constant(1, 0.5), solver_parameters(), answer);

    REQUIRE_FALSE(outcome.has_value());
    CHECK(outcome.error() == refusal::no_solution);
    CHECK(answer.solutions.empty());
    CHECK(answer.iterations.empty());
}

TEST_CASE("a_whole_turn_joint_is_named_inside_its_bounds_before_it_is_projected_onto_one")
{
    const double degree = radians_per_degree;

    CHECK(std::abs(projected_joints(200.0 * degree, 0.0, 0.0)[0] - -160.0 * degree) <= 1.0e-12);
    CHECK(projected_joints(185.0 * degree, 0.0, 0.0)[0] == -170.0 * degree);
    CHECK(projected_joints(180.0 * degree, 0.0, 0.0)[0] == 170.0 * degree);
    CHECK(projected_joints(-180.0 * degree, 0.0, 0.0)[0] == -170.0 * degree);
    CHECK(projected_joints(-185.0 * degree, 0.0, 0.0)[0] == 170.0 * degree);
    CHECK(projected_joints(0.0, 0.5, 0.0)[1] == 0.3);
    CHECK(projected_joints(0.0, -7.0, 0.0)[1] == -0.2);
    CHECK(projected_joints(0.0, 0.1, 9.0)[2] == 9.0);

    for(const joint_vector &once : {projected_joints(200.0 * degree, 0.5, 9.0), projected_joints(185.0 * degree, -0.5, -9.0), projected_joints(180.0 * degree, 0.1, 0.0)})
        CHECK(projected_joints(once[0], once[1], once[2]) == once);
}

TEST_CASE("seeds_a_whole_number_of_turns_past_a_bound_are_named_inside_the_bounds_and_answered_by_both_solves")
{
    for(const auto &[lower, upper, value, named] : whole_turns_past_a_bound)
    {
        CAPTURE(lower, upper, value, named);
        const screw_chain chain      = one_joint_about_z(lower * radians_per_degree, upper * radians_per_degree);
        const joint_vector seed      = joint_vector::Constant(1, value * radians_per_degree);
        const joint_vector projected = projected_inside_bounds(chain, seed);
        CHECK(is_approx_equal(projected[0], named * radians_per_degree, 1.0e-12));
        CHECK(named_inside_bounds(chain, seed) == projected);
        CHECK(inside(chain, projected));
        CHECK(projected_inside_bounds(chain, projected) == projected);
        CHECK(named_inside_bounds(chain, projected) == projected);
        both_solves_answer_at(chain, seed, named * radians_per_degree);
    }
}

TEST_CASE("every_entry_refusal_is_the_one_the_book_solve_answers")
{
    for(const entry &one : entries_the_book_refuses())
        for(const inverse_kinematics_ops inverse : {projected_operations, manipulator::baseline().ik})
        {
            ik_result answer;
            const expected<void, refusal> outcome = solved_through(inverse, one.chain, one.desired, one.seed, tight_parameters, answer);
            REQUIRE_FALSE(outcome.has_value());
            CHECK(outcome.error() == one.kind);
            CHECK(answer.iterations.empty());
        }
}

TEST_CASE("a_joint_without_a_bound_pair_stays_free")
{
    screw_chain chain           = planar_arm();
    chain.limits.lower_position = joint_vector::Constant(1, -1.0);
    chain.limits.upper_position = joint_vector::Constant(1, 1.0);
    const transform target      = pose_of(chain, configuration(0.4, -2.0));

    ik_result answer;
    REQUIRE(solved_through(projected_operations, chain, target, configuration(0.1, -0.1), tight_parameters, answer));

    REQUIRE(answer.solutions.size() == 1u);
    CHECK(is_approx_equal(pose_of(chain, answer.solutions.front()), target, 1.0e-9));
    CHECK(std::abs(answer.solutions.front()[1]) > 1.0);
    CHECK(all_inside(chain, answer.iterations));
}

TEST_CASE("the_first_step_is_the_levenberg_marquardt_step_damped_by_the_error_and_the_bias")
{
    const screw_chain chain = planar_arm();
    const kinematics solver = projected_solver(chain);
    const joint_vector seed = configuration(0.1, -0.1);
    const transform target  = reached(solver, configuration(0.4, -0.7));
    REQUIRE(solver.ik_solve(target, seed, tight_parameters));

    const expected<jacobian, refusal> jb = manipulator::body_jacobian(reference_screw, reference_frames, manipulator::baseline().fk, chain.home, chain.space_screws, seed);
    const expected<std::pair<screw_axis, double>, refusal> logarithm = rigid_motion::matrix_logarithm_se3(transform(rigid_motion::inverse(reached(solver, seed)) * target));
    REQUIRE(jb);
    REQUIRE(logarithm);

    const twist error   = logarithm->first * logarithm->second;
    const double energy = 0.5 * error.dot(weights().asDiagonal() * error);

    const joint_vector &first = solver.iterations().front().joint_positions;
    CHECK(is_approx_equal(first, joint_vector(seed + damped_step(*jb, error, energy + projected::bias)), 1.0e-12));
    CHECK((first - (seed + damped_step(*jb, error, projected::bias))).norm() > 1.0e-9);
}

TEST_CASE("an_unreachable_target_is_refused_with_every_iterate_inside_the_bounds")
{
    screw_chain chain              = planar_arm();
    chain.limits.lower_position[1] = 0.1;
    transform far_away             = transform::Identity();
    far_away(0, 3)                 = 10.0 * (upper_arm + forearm);
    const solver_parameters parameters;

    ik_result answer;
    const expected<void, refusal> outcome = solved_through(projected_operations, chain, far_away, configuration(0.2, 0.2), parameters, answer);

    REQUIRE_FALSE(outcome.has_value());
    CHECK(outcome.error() == refusal::no_solution);
    CHECK(answer.solutions.empty());
    CHECK(answer.iterations.size() <= parameters.max_iterations_per_attempt);
    CHECK(all_inside(chain, answer.iterations));
}

TEST_CASE("a_budget_of_zero_judges_the_seed_and_takes_no_step")
{
    const kinematics solver = projected_solver(planar_arm());

    const expected<joint_vector, refusal> solution = solver.ik_solve(reached(solver, configuration(0.4, -0.7)), configuration(0.1, -0.1), solver_parameters(1.0e-7, 1.0e-7, 0u));

    REQUIRE_FALSE(solution.has_value());
    CHECK(solution.error() == refusal::no_solution);
    CHECK(solver.iterations().empty());
}
