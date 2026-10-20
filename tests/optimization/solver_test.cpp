#include "praxis/optimization/slots.h"
#include "praxis/optimization/problem.h"
#include "praxis/optimization/solver.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/baseline/solver.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::optimization;

namespace {

// g(x) = x^2 - 2 in the book's sign: the error is -g(x) and the Jacobian is dg/dx.
problem square_root_of_two()
{
    return problem{
            .error     = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd::Constant(1, 2.0 - x[0] * x[0]); },
            .jacobian  = [](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal> { return Eigen::MatrixXd::Constant(1, 1, 2.0 * x[0]); },
            .converged = [](const Eigen::VectorXd &e) -> bool { return e.cwiseAbs().maxCoeff() < 1.0e-12; },
    };
}

}

TEST_CASE("the reference solve reaches the root of a problem handed through the slot")
{
    const capabilities composed = baseline();
    const Eigen::VectorXd x0    = Eigen::VectorXd::Constant(1, 1.0);
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> root = composed.solver.solve(square_root_of_two(), x0, 50u, iterates);

    REQUIRE(root);
    REQUIRE(std::abs((*root)[0] - std::sqrt(2.0)) < 1.0e-12);
    REQUIRE(iterates.size() >= 2u);
    REQUIRE(iterates.front() == x0);
    REQUIRE(iterates.back() == *root);
}

namespace {

expected<Eigen::MatrixXd, refusal> unit_slope(const Eigen::VectorXd &)
{
    return Eigen::MatrixXd::Constant(1, 1, 1.0);
}

problem never_converging()
{
    return problem{
            .error     = [](const Eigen::VectorXd &) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd::Constant(1, 1.0); },
            .jacobian  = &unit_slope,
            .converged = [](const Eigen::VectorXd &) -> bool { return false; },
    };
}

problem error_refused_after_the_seed()
{
    return problem{
            .error = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal>
            {
                if(x[0] != 0.0)
                    return praxis::unexpected(refusal::degenerate);
                return Eigen::VectorXd::Constant(1, 1.0);
            },
            .jacobian  = &unit_slope,
            .converged = [](const Eigen::VectorXd &e) -> bool { return e.cwiseAbs().maxCoeff() < 1.0e-12; },
    };
}

// f(x, y) = x^2 + y^2 - 1: one equation in two unknowns, so the zero set is the unit circle.
problem unit_circle()
{
    return problem{
            .error     = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd::Constant(1, 1.0 - x.squaredNorm()); },
            .jacobian  = [](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal> { return Eigen::MatrixXd(2.0 * x.transpose()); },
            .converged = [](const Eigen::VectorXd &e) -> bool { return e.cwiseAbs().maxCoeff() < 1.0e-12; },
    };
}

}

TEST_CASE("a seed at which the problem has converged is answered with the seed alone recorded")
{
    const Eigen::VectorXd x0 = Eigen::VectorXd::Constant(1, std::sqrt(2.0));
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> root = solve(square_root_of_two(), x0, 50u, iterates);

    REQUIRE(root);
    REQUIRE(*root == x0);
    REQUIRE(iterates.size() == 1u);
    REQUIRE(iterates.front() == x0);
}

TEST_CASE("a solve that never converges refuses no_solution at the limit with every iterate kept")
{
    const Eigen::VectorXd x0 = Eigen::VectorXd::Constant(1, 0.5);
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> root = solve(never_converging(), x0, 7u, iterates);

    REQUIRE_FALSE(root);
    REQUIRE(root.error() == refusal::no_solution);
    REQUIRE(iterates.size() == 8u);
    REQUIRE(iterates.front() == x0);
}

TEST_CASE("an error refused after the seed is answered as it came with the iterates reached kept")
{
    const Eigen::VectorXd x0 = Eigen::VectorXd::Zero(1);
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> root = solve(error_refused_after_the_seed(), x0, 50u, iterates);

    REQUIRE_FALSE(root);
    REQUIRE(root.error() == refusal::degenerate);
    REQUIRE(iterates.size() == 2u);
    REQUIRE(iterates.front() == x0);
    REQUIRE(iterates.back() == Eigen::VectorXd::Constant(1, 1.0));
}

TEST_CASE("a jacobian refused at the seed is answered as it came with the seed kept")
{
    problem p                = square_root_of_two();
    p.jacobian               = [](const Eigen::VectorXd &) -> expected<Eigen::MatrixXd, refusal> { return praxis::unexpected(refusal::degenerate); };
    const Eigen::VectorXd x0 = Eigen::VectorXd::Constant(1, 1.0);
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> root = solve(p, x0, 50u, iterates);

    REQUIRE_FALSE(root);
    REQUIRE(root.error() == refusal::degenerate);
    REQUIRE(iterates.size() == 1u);
    REQUIRE(iterates.front() == x0);
}

TEST_CASE("a problem leaving a member the solve reads empty is refused as unsupported input")
{
    problem p  = square_root_of_two();
    p.jacobian = {};
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> root = solve(p, Eigen::VectorXd::Constant(1, 1.0), 50u, iterates);

    REQUIRE_FALSE(root);
    REQUIRE(root.error() == refusal::unsupported_input);
    REQUIRE(iterates.empty());
}

TEST_CASE("an underdetermined problem is solved onto its zero set")
{
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> root = solve(unit_circle(), Eigen::Vector2d(0.3, 0.2), 50u, iterates);

    REQUIRE(root);
    REQUIRE(std::abs(root->squaredNorm() - 1.0) < 1.0e-12);
}

TEST_CASE("the inert solve refuses and the slot reports whether it holds its default")
{
    std::vector<Eigen::VectorXd> iterates;
    const expected<Eigen::VectorXd, refusal> refused = inert::solve(square_root_of_two(), Eigen::VectorXd::Constant(1, 1.0), 50u, iterates);
    const solver_ops unbound{};
    const solver_ops bound{.solve = &solve};
    const capabilities composed                   = baseline();
    const std::array<capability_view, 1> reported = capability_views(composed);

    REQUIRE_FALSE(refused);
    REQUIRE(refused.error() == refusal::not_implemented);
    REQUIRE(holds_default(view_of(unbound), static_cast<std::size_t>(solver_slot::solve)));
    REQUIRE_FALSE(holds_default(view_of(bound), static_cast<std::size_t>(solver_slot::solve)));
    REQUIRE(slot_name(reported[0], 0) == "solver.solve");
    REQUIRE(reported[0].extension() == "optimization");
}

TEST_CASE("the reference solve ignores the bounds a problem carries")
{
    problem bounded          = square_root_of_two();
    bounded.bounds           = box{Eigen::VectorXd::Constant(1, -1.0), Eigen::VectorXd::Zero(1)};
    const Eigen::VectorXd x0 = Eigen::VectorXd::Constant(1, 1.0);
    std::vector<Eigen::VectorXd> free_iterates;
    std::vector<Eigen::VectorXd> bounded_iterates;

    const expected<Eigen::VectorXd, refusal> free_root    = solve(square_root_of_two(), x0, 50u, free_iterates);
    const expected<Eigen::VectorXd, refusal> bounded_root = solve(bounded, x0, 50u, bounded_iterates);

    REQUIRE(free_root);
    REQUIRE(bounded_root);
    REQUIRE(*free_root == *bounded_root);
    REQUIRE(free_iterates == bounded_iterates);
}
