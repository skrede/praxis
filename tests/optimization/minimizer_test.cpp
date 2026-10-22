#include "praxis/optimization/slots.h"
#include "praxis/optimization/solver.h"
#include "praxis/optimization/problem.h"
#include "praxis/optimization/minimizer.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/baseline/solver.h"
#include "praxis/optimization/baseline/minimizer.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::optimization;

namespace {

constexpr minimizer_ops references{.newton = &newton, .gradient_descent = &gradient_descent};

constexpr auto gradient_vanishes = [](const Eigen::VectorXd &g) -> bool { return g.cwiseAbs().maxCoeff() < 1.0e-12; };

// f(x) = e^x - 2x, whose one minimizer is ln 2.
problem exponential_bowl()
{
    return problem{
            .objective  = [](const Eigen::VectorXd &x) -> expected<double, refusal> { return std::exp(x[0]) - 2.0 * x[0]; },
            .gradient   = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd::Constant(1, std::exp(x[0]) - 2.0); },
            .hessian    = [](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal> { return Eigen::MatrixXd::Constant(1, 1, std::exp(x[0])); },
            .stationary = gradient_vanishes,
    };
}

// f(x, y) = (x^2 + 10 y^2) / 2.
problem elongated_bowl()
{
    return problem{
            .gradient   = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd(Eigen::Vector2d(x[0], 10.0 * x[1])); },
            .hessian    = [](const Eigen::VectorXd &) -> expected<Eigen::MatrixXd, refusal> { return Eigen::MatrixXd(Eigen::Vector2d(1.0, 10.0).asDiagonal()); },
            .stationary = gradient_vanishes,
    };
}

}

TEST_CASE("the reference Newton minimizer reaches the minimizer of an objective handed through the slot")
{
    const capabilities composed = baseline();
    const Eigen::VectorXd x0    = Eigen::VectorXd::Zero(1);
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> minimizer = composed.minimizer.newton(exponential_bowl(), x0, 50u, iterates);

    REQUIRE(minimizer);
    REQUIRE(std::abs((*minimizer)[0] - std::log(2.0)) < 1.0e-12);
    REQUIRE(iterates.front() == x0);
    REQUIRE(iterates.back() == *minimizer);
}

TEST_CASE("a seed at which the problem is stationary is answered at once with only the seed kept by either minimizer")
{
    const Eigen::VectorXd x0 = Eigen::Vector2d::Zero();
    std::vector<Eigen::VectorXd> newton_iterates;
    std::vector<Eigen::VectorXd> descent_iterates;

    const expected<Eigen::VectorXd, refusal> by_newton  = references.newton(elongated_bowl(), x0, 50u, newton_iterates);
    const expected<Eigen::VectorXd, refusal> by_descent = references.gradient_descent(elongated_bowl(), x0, 0.05, 50u, descent_iterates);

    REQUIRE(by_newton);
    REQUIRE(by_descent);
    CHECK(*by_newton == x0);
    CHECK(*by_descent == x0);
    CHECK(newton_iterates == std::vector<Eigen::VectorXd>{x0});
    CHECK(descent_iterates == std::vector<Eigen::VectorXd>{x0});
}

TEST_CASE("Newton answers whatever stationary point its seed leads to, a local maximum and a saddle included")
{
    const problem quartic{
            .gradient   = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd::Constant(1, x[0] * x[0] * x[0] - x[0]); },
            .hessian    = [](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal> { return Eigen::MatrixXd::Constant(1, 1, 3.0 * x[0] * x[0] - 1.0); },
            .stationary = gradient_vanishes,
    };
    const problem saddle{
            .gradient   = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd(Eigen::Vector2d(2.0 * x[0], -2.0 * x[1])); },
            .hessian    = [](const Eigen::VectorXd &) -> expected<Eigen::MatrixXd, refusal> { return Eigen::MatrixXd(Eigen::Vector2d(2.0, -2.0).asDiagonal()); },
            .stationary = gradient_vanishes,
    };
    std::vector<Eigen::VectorXd> quartic_iterates;
    std::vector<Eigen::VectorXd> saddle_iterates;

    const expected<Eigen::VectorXd, refusal> maximum = references.newton(quartic, Eigen::VectorXd::Constant(1, 0.1), 50u, quartic_iterates);
    const expected<Eigen::VectorXd, refusal> crossed = references.newton(saddle, Eigen::Vector2d(0.3, 0.2), 50u, saddle_iterates);

    REQUIRE(maximum);
    REQUIRE(crossed);
    CHECK(std::abs((*maximum)[0]) < 1.0e-12);
    CHECK(crossed->cwiseAbs().maxCoeff() < 1.0e-12);
    CHECK(saddle_iterates.size() == 2u);
}

TEST_CASE("Newton answers the minimizer of a quadratic bowl in one step from any seed")
{
    for(const Eigen::Vector2d &seed : {Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d(-3.0, 0.5), Eigen::Vector2d(100.0, -40.0)})
    {
        std::vector<Eigen::VectorXd> iterates;

        const expected<Eigen::VectorXd, refusal> answered = references.newton(elongated_bowl(), seed, 50u, iterates);

        REQUIRE(answered);
        CHECK(answered->cwiseAbs().maxCoeff() < 1.0e-12);
        CHECK(iterates.size() == 2u);
    }
}

TEST_CASE("fixed-step gradient descent answers the minimizer of a quadratic bowl at a step length below two over its largest curvature")
{
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> answered = references.gradient_descent(elongated_bowl(), Eigen::Vector2d(1.0, 1.0), 0.05, 1000u, iterates);

    REQUIRE(answered);
    CHECK(answered->cwiseAbs().maxCoeff() < 1.0e-12);
    CHECK(iterates.back() == *answered);
}

TEST_CASE("gradient descent answers without a hessian and never calls one the problem carries")
{
    const problem without_hessian{.gradient = elongated_bowl().gradient, .stationary = gradient_vanishes};
    std::size_t calls = 0;
    problem counted   = elongated_bowl();
    counted.hessian   = [&calls](const Eigen::VectorXd &) -> expected<Eigen::MatrixXd, refusal>
    {
        ++calls;
        return Eigen::MatrixXd(Eigen::Vector2d(1.0, 10.0).asDiagonal());
    };
    std::vector<Eigen::VectorXd> bare_iterates;
    std::vector<Eigen::VectorXd> counted_iterates;

    const expected<Eigen::VectorXd, refusal> bare     = references.gradient_descent(without_hessian, Eigen::Vector2d(1.0, 1.0), 0.05, 1000u, bare_iterates);
    const expected<Eigen::VectorXd, refusal> answered = references.gradient_descent(counted, Eigen::Vector2d(1.0, 1.0), 0.05, 1000u, counted_iterates);

    REQUIRE(bare);
    REQUIRE(answered);
    CHECK(calls == 0u);
    CHECK(bare_iterates == counted_iterates);
}

TEST_CASE("the inert minimizers refuse and each minimizer slot reports whether it holds its default")
{
    std::vector<Eigen::VectorXd> iterates;
    const expected<Eigen::VectorXd, refusal> by_newton  = inert::newton(elongated_bowl(), Eigen::Vector2d(1.0, 1.0), 50u, iterates);
    const expected<Eigen::VectorXd, refusal> by_descent = inert::gradient_descent(elongated_bowl(), Eigen::Vector2d(1.0, 1.0), 0.05, 50u, iterates);
    const minimizer_ops unbound{};
    const minimizer_ops bound                     = references;
    const capabilities composed                   = baseline();
    const std::array<capability_view, 2> reported = capability_views(composed);

    REQUIRE_FALSE(by_newton);
    REQUIRE_FALSE(by_descent);
    CHECK(by_newton.error() == refusal::not_implemented);
    CHECK(by_descent.error() == refusal::not_implemented);
    CHECK(iterates.empty());
    CHECK(holds_default(view_of(unbound), static_cast<std::size_t>(minimizer_slot::newton)));
    CHECK(holds_default(view_of(unbound), static_cast<std::size_t>(minimizer_slot::gradient_descent)));
    CHECK_FALSE(holds_default(view_of(bound), static_cast<std::size_t>(minimizer_slot::newton)));
    CHECK_FALSE(holds_default(view_of(bound), static_cast<std::size_t>(minimizer_slot::gradient_descent)));
    CHECK(slot_name(reported[1], 0) == "minimizer.newton");
    CHECK(slot_name(reported[1], 1) == "minimizer.gradient_descent");
    CHECK(reported[1].extension() == "optimization");
}

TEST_CASE("the reference solve answers identically whether or not a problem carries the objective form")
{
    const problem residual{
            .error     = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd::Constant(1, 2.0 - x[0] * x[0]); },
            .jacobian  = [](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal> { return Eigen::MatrixXd::Constant(1, 1, 2.0 * x[0]); },
            .converged = [](const Eigen::VectorXd &e) -> bool { return e.cwiseAbs().maxCoeff() < 1.0e-12; },
    };
    problem both    = residual;
    both.objective  = [](const Eigen::VectorXd &) -> expected<double, refusal> { return praxis::unexpected(refusal::degenerate); };
    both.gradient   = [](const Eigen::VectorXd &) -> expected<Eigen::VectorXd, refusal> { return praxis::unexpected(refusal::degenerate); };
    both.hessian    = [](const Eigen::VectorXd &) -> expected<Eigen::MatrixXd, refusal> { return praxis::unexpected(refusal::degenerate); };
    both.stationary = [](const Eigen::VectorXd &) -> bool { return false; };
    std::vector<Eigen::VectorXd> residual_iterates;
    std::vector<Eigen::VectorXd> both_iterates;

    const expected<Eigen::VectorXd, refusal> residual_root = solve(residual, Eigen::VectorXd::Constant(1, 1.0), 50u, residual_iterates);
    const expected<Eigen::VectorXd, refusal> both_root     = solve(both, Eigen::VectorXd::Constant(1, 1.0), 50u, both_iterates);

    REQUIRE(residual_root);
    REQUIRE(both_root);
    CHECK(*residual_root == *both_root);
    CHECK(residual_iterates == both_iterates);
}
