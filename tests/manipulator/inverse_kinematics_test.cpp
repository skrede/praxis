#include "captured_log.h"
#include "recorded_chain.h"
#include "two_joint_bindings.h"

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/evaluation/tolerance.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <span>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::fixture;

namespace {

template<typename T>
bool refused(const expected<T, refusal> &answer, refusal reason)
{
    return !answer.has_value() && answer.error() == reason;
}

kinematics deployed_arm(screw_chain chain)
{
    const capabilities reference = baseline();
    expected<kinematics, refusal> composed =
            kinematics::compose(std::move(chain), reference.fk, reference.dk, reference.ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame);
    REQUIRE(composed);

    return std::move(*composed);
}

}

// What makes a solve substitutable: it reads forward kinematics off what the composition bound, so
// the composition decides which one it sees rather than the solve carrying its own.
TEST_CASE("a solve asks the composition for forward kinematics rather than computing its own")
{
    const kinematics composed =
            holding(forward_kinematics_ops{.forward_kinematics = &lifting_forward_kinematics}, {}, inverse_kinematics_ops{.inverse_kinematics = &fk_reading_inverse_kinematics});
    const kinematics alone = holding({}, {}, inverse_kinematics_ops{.inverse_kinematics = &fk_reading_inverse_kinematics});
    const joint_vector j0  = joint_vector::Constant(2, 0.75);

    const expected<joint_vector, refusal> read = composed.ik_solve(transform::Identity(), j0, solver_parameters());
    REQUIRE(read.has_value());
    CHECK(is_approx_equal((*read)[0], 0.75));

    const expected<joint_vector, refusal> unread = alone.ik_solve(transform::Identity(), j0, solver_parameters());
    REQUIRE_FALSE(unread.has_value());
    CHECK(unread.error() == refusal::not_implemented);
}

TEST_CASE("iterations are reported only by a solve that records them")
{
    const kinematics quiet = holding({}, {}, inverse_kinematics_ops{.inverse_kinematics = &two_solution_inverse_kinematics});
    const kinematics recording =
            holding(forward_kinematics_ops{.forward_kinematics = &lifting_forward_kinematics}, {}, inverse_kinematics_ops{.inverse_kinematics = &fk_reading_inverse_kinematics});
    const joint_vector j0 = joint_vector::Constant(2, 0.75);

    const expected<joint_vector, refusal> from_quiet = quiet.ik_solve(transform::Identity(), j0, solver_parameters());
    REQUIRE(from_quiet.has_value());
    CHECK(is_approx_equal((*from_quiet)[0], 1.0));
    CHECK(quiet.iterations().empty());

    REQUIRE(recording.ik_solve(transform::Identity(), j0, solver_parameters()).has_value());
    REQUIRE(recording.iterations().size() == 1u);
    CHECK(recording.iterations().front().index == 0u);
    CHECK(is_approx_equal(recording.iterations().front().angular_error, 0.5));
    CHECK(is_approx_equal(recording.iterations().front().linear_error, 0.125));
}

TEST_CASE("the selector chooses among the solutions a solve produced")
{
    const kinematics solver = holding({}, {}, inverse_kinematics_ops{.inverse_kinematics = &two_solution_inverse_kinematics});
    const joint_vector j0   = joint_vector::Constant(2, 0.0);
    const selector second   = [](std::span<const joint_vector>) { return std::optional<std::size_t>(1u); };

    const expected<joint_vector, refusal> chosen = solver.ik_solve(transform::Identity(), j0, solver_parameters(), second);
    REQUIRE(chosen.has_value());
    CHECK(is_approx_equal((*chosen)[0], 2.0));
}

// A selection naming nothing and one naming a candidate that does not exist are different failures,
// and neither is the first candidate.
TEST_CASE("a selection that names no candidate is refused separately from one outside the range")
{
    const kinematics solver = holding({}, {}, inverse_kinematics_ops{.inverse_kinematics = &two_solution_inverse_kinematics});
    const joint_vector j0   = joint_vector::Constant(2, 0.0);

    const selector vetoing      = [](std::span<const joint_vector>) { return std::optional<std::size_t>(); };
    const selector past_the_end = [](std::span<const joint_vector>) { return std::optional<std::size_t>(7u); };

    const expected<joint_vector, refusal> vetoed = solver.ik_solve(transform::Identity(), j0, solver_parameters(), vetoing);
    const expected<joint_vector, refusal> beyond = solver.ik_solve(transform::Identity(), j0, solver_parameters(), past_the_end);

    REQUIRE_FALSE(vetoed.has_value());
    REQUIRE_FALSE(beyond.has_value());
    CHECK(vetoed.error() == refusal::no_solution);
    CHECK(beyond.error() == refusal::degenerate);
}

TEST_CASE("a solve that converges on nothing refuses rather than answering with the seed")
{
    const kinematics solver = holding({}, {}, inverse_kinematics_ops{.inverse_kinematics = &converging_on_nothing});
    const joint_vector j0   = joint_vector::Constant(2, 0.4);
    const selector anyhow   = [](std::span<const joint_vector>) { return std::optional<std::size_t>(0u); };

    const expected<joint_vector, refusal> plain  = solver.ik_solve(transform::Identity(), j0, solver_parameters());
    const expected<joint_vector, refusal> picked = solver.ik_solve(transform::Identity(), j0, solver_parameters(), anyhow);

    REQUIRE_FALSE(plain.has_value());
    REQUIRE_FALSE(picked.has_value());
    CHECK(plain.error() == refusal::no_solution);
    CHECK(picked.error() == refusal::no_solution);
}

// Three values distinct in magnitude and in order, so an argument mapped onto the wrong field shows
// up as a mismatch rather than as an equality that happened to hold.
TEST_CASE("the solver parameters carry the three quantities the stopping test uses and no others")
{
    const solver_parameters spelled_out(1.0e-7, 1.0e-3, 91u);

    CHECK(is_approx_equal(spelled_out.position_tol, 1.0e-7));
    CHECK(is_approx_equal(spelled_out.orientation_tol, 1.0e-3));
    CHECK(spelled_out.max_iterations_per_attempt == 91u);

    const solver_parameters defaulted;
    CHECK(defaulted.position_tol > 0.0);
    CHECK(defaulted.orientation_tol > 0.0);
    CHECK(defaulted.max_iterations_per_attempt > 0u);
}

TEST_CASE("binding one of the two solves is never the price of binding the other")
{
    const inverse_kinematics_ops searching{.inverse_kinematics = &two_solution_inverse_kinematics};
    const inverse_kinematics_ops closed{.analytic_inverse_kinematics = &one_answer_analytic_inverse_kinematics};
    const inverse_kinematics_ops both{.inverse_kinematics = &two_solution_inverse_kinematics, .analytic_inverse_kinematics = &one_answer_analytic_inverse_kinematics};
    const joint_vector j0 = joint_vector::Constant(2, 0.0);

    const kinematics searches = holding({}, {}, searching);
    const kinematics answers  = holding({}, {}, closed);
    const kinematics does     = holding({}, {}, both);
    const kinematics neither  = holding({}, {}, {});

    CHECK(searches.ik_solve(transform::Identity(), j0, solver_parameters()).has_value());
    CHECK(refused(searches.configurations_reaching(transform::Identity()), refusal::not_implemented));
    CHECK(answers.configurations_reaching(transform::Identity()).has_value());
    CHECK(refused(answers.ik_solve(transform::Identity(), j0, solver_parameters()), refusal::not_implemented));
    CHECK(does.ik_solve(transform::Identity(), j0, solver_parameters()).has_value());
    CHECK(does.configurations_reaching(transform::Identity()).has_value());
    CHECK(refused(neither.ik_solve(transform::Identity(), j0, solver_parameters()), refusal::not_implemented));
    CHECK(refused(neither.configurations_reaching(transform::Identity()), refusal::not_implemented));
}

TEST_CASE("an answer taken in one go carries no iterates and is an entry into the solve all the same")
{
    const kinematics answers = holding({}, {}, inverse_kinematics_ops{.analytic_inverse_kinematics = &one_answer_analytic_inverse_kinematics});

    const expected<std::span<const joint_vector>, refusal> answered = answers.configurations_reaching(transform::Identity());

    REQUIRE(answered.has_value());
    CHECK(answered->size() == 1u);
    CHECK(answers.solutions().size() == 1u);
    CHECK(answers.iterations().empty());
    CHECK(answers.solve_count() == 1u);
}

TEST_CASE("the closed form answers several configurations of the deployed arm for one pose, each of them at it")
{
    const kinematics arm                      = deployed_arm(recorded_arm());
    const joint_vector stood                  = recorded_configuration(0.3, -0.7, 0.9, 0.4, 0.8, -0.2);
    const expected<transform, refusal> target = arm.fk_solve(stood);
    REQUIRE(target.has_value());

    const expected<std::span<const joint_vector>, refusal> answered = arm.configurations_reaching(*target);

    REQUIRE(answered.has_value());
    CHECK(answered->size() > 1u);
    CHECK(answered->size() <= 8u);
    for(const joint_vector &configuration : *answered)
    {
        const expected<transform, refusal> reached = arm.fk_solve(configuration);
        REQUIRE(reached.has_value());
        CHECK(is_approx_equal(*reached, *target, 1.0e-6));
    }
    CHECK(arm.iterations().empty());
}

// The arm's second joint runs from -190 to +45 degrees, which is more than half a turn below zero, so
// a posture standing past -180 degrees there is one the closed form names at the turn above it. The
// naming a chain's bounds admit is the one answered.
TEST_CASE("a posture the bounds admit only a whole turn from where the closed form names it is answered there")
{
    const kinematics arm                      = deployed_arm(recorded_arm());
    const joint_vector wound                  = recorded_configuration(0.3, -3.2, 0.9, 0.4, 0.8, -0.2);
    const expected<transform, refusal> target = arm.fk_solve(wound);
    REQUIRE(target.has_value());

    const expected<std::span<const joint_vector>, refusal> answered = arm.configurations_reaching(*target);

    REQUIRE(answered.has_value());
    CHECK(std::any_of(answered->begin(), answered->end(), [&wound](const joint_vector &named) { return is_approx_equal(named, wound, 1.0e-9); }));
    for(const joint_vector &configuration : *answered)
    {
        CHECK(configuration[1] >= recorded_lower_bounds()[1]);
        CHECK(configuration[1] <= recorded_upper_bounds()[1]);

        const expected<transform, refusal> reached = arm.fk_solve(configuration);
        REQUIRE(reached.has_value());
        CHECK(is_approx_equal(*reached, *target, 1.0e-6));
    }
}

// An arm whose wrist axes miss each other is the kind the closed form has no decomposition for, and
// the smaller machine the demonstration ships is one.
TEST_CASE("a chain the closed form cannot take is refused by the slot and named where it was asked")
{
    const tests::captured_log recorded;
    screw_chain offset_wrist = recorded_arm();
    offset_wrist.space_screws[4] << 0.0, 1.0, 0.0, -0.520, 0.0, 0.900;

    const kinematics arm                                            = deployed_arm(std::move(offset_wrist));
    const expected<std::span<const joint_vector>, refusal> answered = arm.configurations_reaching(recorded_home());

    CHECK(refused(answered, refusal::unsupported_input));
    CHECK_THAT(recorded.text(), Catch::Matchers::ContainsSubstring("ik.analytic_inverse_kinematics"));
}
