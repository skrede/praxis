#include "two_joint_bindings.h"

#include "praxis/manipulator/kinematics.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <cstddef>
#include <optional>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::fixture;

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
