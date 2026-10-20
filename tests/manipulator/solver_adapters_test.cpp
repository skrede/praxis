#include "captured_log.h"

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/solver_adapters.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/evaluation/comparators.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/optimization/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <utility>
#include <optional>
#include <algorithm>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

constexpr double upper_arm = 0.5;
constexpr double forearm   = 0.4;
constexpr double unbounded = std::numeric_limits<double>::infinity();

// A planar two-link arm about the world z, the tool at the far end of the second link. Lynch & Park,
// Modern Robotics, example 4.3: a revolute screw axis is (w, -w x q) for a point q on the axis.
screw_chain planar_arm(Eigen::Index limited = 2)
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
    bounds.lower_position = joint_vector::Constant(limited, -3.0);
    bounds.upper_position = joint_vector::Constant(limited, 3.0);

    return screw_chain(home, {shoulder, elbow}, bounds);
}

joint_vector configuration(double first, double second)
{
    joint_vector q(2);
    q << first, second;

    return q;
}

const solver_parameters tight(1.0e-10, 1.0e-10, 100u);
const joint_vector seed = configuration(0.3, -0.5);

transform target()
{
    return forward_kinematics(rigid_motion::baseline().screw, planar_arm().home, planar_arm().space_screws, configuration(0.6, -0.9)).value();
}

using adapter = decltype(inverse_kinematics_ops::inverse_kinematics);
constexpr std::array<adapter, 2> adapters{&body_inverse_kinematics, &space_inverse_kinematics};

bool space_columns_asked = false;
bool body_columns_asked  = false;

expected<jacobian, refusal> marked_space_jacobian(const rigid_motion::screw_ops &screw, std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    space_columns_asked = true;
    return space_jacobian(screw, space_screws, theta);
}

expected<jacobian, refusal> marked_body_jacobian(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward, const transform &m,
                                                 std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    body_columns_asked = true;
    return body_jacobian(screw, frames, forward, m, space_screws, theta);
}

Eigen::Index answered_width = 2;

expected<jacobian, refusal> of_answered_width(const expected<jacobian, refusal> &full)
{
    if(!full)
        return full;

    jacobian resized        = jacobian::Zero(6, answered_width);
    const Eigen::Index kept = std::min(answered_width, full->cols());
    resized.leftCols(kept)  = full->leftCols(kept);
    return resized;
}

expected<jacobian, refusal> resized_space_jacobian(const rigid_motion::screw_ops &screw, std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    return of_answered_width(space_jacobian(screw, space_screws, theta));
}

expected<jacobian, refusal> resized_body_jacobian(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward, const transform &m,
                                                  std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    return of_answered_width(body_jacobian(screw, frames, forward, m, space_screws, theta));
}

expected<transform, refusal> refusing_past_the_seed(const rigid_motion::screw_ops &screw, const transform &m, std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    if(theta != seed)
        return unexpected(refusal::degenerate);
    return forward_kinematics(screw, m, space_screws, theta);
}

struct seen_solve
{
    optimization::problem problem;
    std::vector<Eigen::VectorXd> iterates;
    std::optional<refusal> refused;
};

seen_solve seen;

expected<Eigen::VectorXd, refusal> recording_reference(const optimization::problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    const expected<Eigen::VectorXd, refusal> root = optimization::baseline().solver.solve(p, x0, max_iterations, iterates);
    seen                                          = seen_solve{p, iterates, root ? std::nullopt : std::optional<refusal>(root.error())};
    return root;
}

expected<Eigen::VectorXd, refusal> answering_a_narrower_root(const optimization::problem &, const Eigen::VectorXd &x0, std::uint32_t, std::vector<Eigen::VectorXd> &iterates)
{
    iterates.push_back(x0);
    return Eigen::VectorXd(x0.head(1));
}

// Answers its last iterate whether or not the error converged there.
expected<Eigen::VectorXd, refusal> answering_its_last_iterate(const optimization::problem &p, const Eigen::VectorXd &x0, std::uint32_t, std::vector<Eigen::VectorXd> &iterates)
{
    static_cast<void>(optimization::baseline().solver.solve(p, x0, 1u, iterates));
    return iterates.back();
}

expected<void, refusal> asked(adapter solve, const forward_kinematics_ops &forward, optimization::solver_ops solver, const screw_chain &chain, const transform &desired,
                              const joint_vector &j0, ik_result &answer, const differential_kinematics_ops &differential = manipulator::baseline().dk)
{
    return solve(rigid_motion::baseline().screw, rigid_motion::baseline().frame, forward, differential, solver, chain, desired, j0, tight, answer);
}

void reaches_the_target(inverse_kinematics_ops inverse, differential_kinematics_ops differential)
{
    expected<kinematics, refusal> solver = kinematics::compose(planar_arm(), manipulator::baseline().fk, differential, inverse, rigid_motion::baseline().screw,
                                                               rigid_motion::baseline().frame, optimization::baseline().solver);
    REQUIRE(solver);

    const expected<joint_vector, refusal> solved = solver->ik_solve(target(), seed, tight);
    REQUIRE(solved);
    const evaluation::residual apart = evaluation::pose_residual(solver->fk_solve(*solved).value(), target());
    CHECK(apart.magnitude <= 1.0e-6);
    CHECK(apart.linear_error_metres <= 1.0e-6);

    REQUIRE_FALSE(solver->iterations().empty());
    CHECK(solver->iterations().front().joint_positions == seed);
    CHECK(solver->iterations().front().step_norm == 0.0);
    CHECK(solver->iterations().front().index == 0u);
}

const differential_kinematics_ops marked_columns{.space_jacobian = &marked_space_jacobian, .body_jacobian = &marked_body_jacobian};
const differential_kinematics_ops resized_columns{.space_jacobian = &resized_space_jacobian, .body_jacobian = &resized_body_jacobian};

}

TEST_CASE("the body adapter driven by the reference solver reaches the target")
{
    reaches_the_target(inverse_kinematics_ops{&body_inverse_kinematics}, manipulator::baseline().dk);
}

TEST_CASE("the body adapter asks the body Jacobian and never the space one")
{
    space_columns_asked = body_columns_asked = false;
    reaches_the_target(inverse_kinematics_ops{&body_inverse_kinematics}, marked_columns);
    CHECK(body_columns_asked);
    CHECK_FALSE(space_columns_asked);
}

TEST_CASE("the space adapter reaches the same target asking the space Jacobian and never the body one")
{
    space_columns_asked = body_columns_asked = false;
    reaches_the_target(inverse_kinematics_ops{&space_inverse_kinematics}, marked_columns);
    CHECK(space_columns_asked);
    CHECK_FALSE(body_columns_asked);
}

TEST_CASE("the problem is bounded by the chain's joint positions and free where the limits do not cover a joint")
{
    for(const adapter solve : adapters)
    {
        ik_result answer;
        REQUIRE(asked(solve, manipulator::baseline().fk, {&recording_reference}, planar_arm(1), target(), seed, answer));
        REQUIRE(seen.problem.bounds.has_value());
        CHECK(seen.problem.bounds->lower == configuration(-3.0, -unbounded));
        CHECK(seen.problem.bounds->upper == configuration(3.0, unbounded));
    }
}

TEST_CASE("the problem wraps a joint that turns whole by a turn and neither a sliding nor a pitched one")
{
    constexpr double turn = 2.0 * std::numbers::pi;
    screw_chain sliding   = planar_arm();
    sliding.space_screws[1] << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    screw_chain pitched = planar_arm();
    pitched.space_screws[1] << 0.0, 0.0, 1.0, 0.0, -upper_arm, 0.1;

    for(const adapter solve : adapters)
        for(const auto &[chain, second] : {std::pair{planar_arm(1), turn}, std::pair{sliding, 0.0}, std::pair{pitched, 0.0}})
        {
            seen = {};
            ik_result answer;
            static_cast<void>(asked(solve, manipulator::baseline().fk, {&recording_reference}, chain, target(), seed, answer));
            REQUIRE(seen.problem.bounds.has_value());
            CHECK(seen.problem.bounds->period == configuration(turn, second));
        }
}

TEST_CASE("handed the inert solver an adapter refuses not_implemented, names no configuration and says the solver is unbound")
{
    for(const adapter solve : adapters)
    {
        ik_result answer;
        expected<void, refusal> solved;
        const std::string said = tests::reported_by([&] { solved = asked(solve, manipulator::baseline().fk, optimization::solver_ops{}, planar_arm(), target(), seed, answer); });
        REQUIRE_FALSE(solved);
        CHECK(solved.error() == refusal::not_implemented);
        CHECK(answer.solutions.empty());
        CHECK(said.find("'solver.solve' is not implemented") != std::string::npos);
    }
}

TEST_CASE("a bound Jacobian of another width than the seed is refused before a step is taken through it")
{
    for(const adapter solve : adapters)
        for(const Eigen::Index width : {Eigen::Index{3}, Eigen::Index{1}})
        {
            answered_width = width;
            seen           = {};
            ik_result answer;
            const expected<void, refusal> solved = asked(solve, manipulator::baseline().fk, {&recording_reference}, planar_arm(), target(), seed, answer, resized_columns);
            REQUIRE_FALSE(solved);
            CHECK(solved.error() == refusal::no_solution);
            CHECK(seen.refused == refusal::degenerate);
            CHECK(seen.iterates.size() == 1u);
            CHECK(answer.solutions.empty());
        }
}

TEST_CASE("a root the solver answers where the problem has not converged is answered no_solution")
{
    for(const adapter solve : adapters)
    {
        ik_result answer;
        const expected<void, refusal> solved = asked(solve, manipulator::baseline().fk, {&answering_its_last_iterate}, planar_arm(), target(), seed, answer);
        REQUIRE_FALSE(solved);
        CHECK(solved.error() == refusal::no_solution);
        CHECK(answer.solutions.empty());
        CHECK(answer.iterations.size() == 2u);
    }
}

TEST_CASE("a forward map refusing past the seed is answered no_solution with the iterates reached")
{
    forward_kinematics_ops forward = manipulator::baseline().fk;
    forward.forward_kinematics     = &refusing_past_the_seed;
    for(const adapter solve : adapters)
    {
        ik_result answer;
        const expected<void, refusal> solved = asked(solve, forward, {&recording_reference}, planar_arm(), target(), seed, answer);
        REQUIRE_FALSE(solved);
        CHECK(solved.error() == refusal::no_solution);
        CHECK(seen.refused == refusal::degenerate);
        CHECK(answer.solutions.empty());
        REQUIRE(answer.iterations.size() == 2u);
        CHECK(std::isfinite(answer.iterations[0].angular_error));
        CHECK(std::isnan(answer.iterations[1].angular_error));
        CHECK(std::isnan(answer.iterations[1].linear_error));
    }
}

TEST_CASE("an adapter refuses at entry in the order the reference does")
{
    for(const adapter solve : adapters)
    {
        ik_result answer;
        const joint_vector unread = configuration(std::numeric_limits<double>::quiet_NaN(), 0.0);
        CHECK(asked(solve, manipulator::baseline().fk, {}, planar_arm(), target(), joint_vector::Constant(3, std::nan("")), answer).error() == refusal::unsupported_input);
        CHECK(asked(solve, manipulator::baseline().fk, {}, planar_arm(), transform(2.0 * target()), unread, answer).error() == refusal::degenerate);
        CHECK(asked(solve, manipulator::baseline().fk, {}, planar_arm(), target(), unread, answer).error() == refusal::no_solution);
        CHECK(asked(solve, manipulator::baseline().fk, {&answering_a_narrower_root}, planar_arm(), target(), seed, answer).error() == refusal::no_solution);
        CHECK(answer.solutions.empty());
    }
}

TEST_CASE("each recorded error is the problem's own error at that iterate")
{
    for(const adapter solve : adapters)
    {
        ik_result answer;
        REQUIRE(asked(solve, manipulator::baseline().fk, {&recording_reference}, planar_arm(), target(), seed, answer));
        REQUIRE(answer.iterations.size() == seen.iterates.size());
        REQUIRE(answer.solutions.size() == 1u);
        CHECK(answer.solutions.front() == seen.iterates.back());
        for(std::size_t k = 0; k < seen.iterates.size(); ++k)
        {
            const Eigen::VectorXd e = seen.problem.error(seen.iterates[k]).value();
            CHECK(answer.iterations[k].joint_positions == seen.iterates[k]);
            CHECK(answer.iterations[k].angular_error == e.head<3>().norm());
            CHECK(answer.iterations[k].linear_error == e.tail<3>().norm());
            CHECK(answer.iterations[k].step_norm == (k == 0 ? 0.0 : (seen.iterates[k] - seen.iterates[k - 1]).norm()));
            CHECK(answer.iterations[k].index == k);
        }
    }
}
