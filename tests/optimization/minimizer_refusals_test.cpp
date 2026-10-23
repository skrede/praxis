#include "praxis/optimization/problem.h"
#include "praxis/optimization/minimizer.h"
#include "praxis/optimization/baseline/minimizer.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::optimization;

namespace {

constexpr minimizer_ops references{.newton = &newton, .gradient_descent = &gradient_descent};

constexpr auto gradient_vanishes = [](const Eigen::VectorXd &g) -> bool { return g.cwiseAbs().maxCoeff() < 1.0e-12; };

// f(x, y) = (x^2 + 10 y^2) / 2.
problem elongated_bowl()
{
    return problem{
            .gradient   = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd(Eigen::Vector2d(x[0], 10.0 * x[1])); },
            .hessian    = [](const Eigen::VectorXd &) -> expected<Eigen::MatrixXd, refusal> { return Eigen::MatrixXd(Eigen::Vector2d(1.0, 10.0).asDiagonal()); },
            .stationary = gradient_vanishes,
    };
}

problem refusing_after_seed(refusal kind, std::size_t &calls)
{
    problem refusing  = elongated_bowl();
    refusing.gradient = [&calls, kind](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal>
    {
        if(calls++ > 0)
            return praxis::unexpected(kind);
        return Eigen::VectorXd(Eigen::Vector2d(x[0], 10.0 * x[1]));
    };
    return refusing;
}

void check_gradient_refusal(refusal kind)
{
    INFO("refused as " << static_cast<int>(kind));
    std::size_t calls        = 0;
    const problem refusing   = refusing_after_seed(kind, calls);
    const Eigen::VectorXd x0 = Eigen::Vector2d(1.0, 1.0);
    std::vector<Eigen::VectorXd> newton_iterates;
    std::vector<Eigen::VectorXd> descent_iterates;

    const expected<Eigen::VectorXd, refusal> by_newton  = references.newton(refusing, x0, 50u, newton_iterates);
    calls                                               = 0;
    const expected<Eigen::VectorXd, refusal> by_descent = references.gradient_descent(refusing, x0, 0.05, 50u, descent_iterates);

    REQUIRE_FALSE(by_newton);
    REQUIRE_FALSE(by_descent);
    CHECK(by_newton.error() == kind);
    CHECK(by_descent.error() == kind);
    REQUIRE(newton_iterates.size() == 2u);
    REQUIRE(descent_iterates.size() == 2u);
    CHECK(newton_iterates.front() == x0);
    CHECK(descent_iterates.front() == x0);
    CHECK(newton_iterates.back() != x0);
    CHECK(descent_iterates.back() != x0);
}

}

TEST_CASE("a fixed-step descent above two over the largest curvature is refused no_solution at its limit with every iterate kept")
{
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> answered = references.gradient_descent(elongated_bowl(), Eigen::Vector2d(1.0, 1.0), 0.25, 200u, iterates);

    REQUIRE_FALSE(answered);
    CHECK(answered.error() == refusal::no_solution);
    REQUIRE(iterates.size() == 201u);
    for(std::size_t k = 1; k < iterates.size(); ++k)
        CHECK(std::abs(iterates[k][1]) > std::abs(iterates[k - 1][1]));
}

TEST_CASE("a stationary test that never holds is refused no_solution at the limit with the seed and every step kept by either minimizer")
{
    problem never    = elongated_bowl();
    never.stationary = [](const Eigen::VectorXd &) -> bool { return false; };
    std::vector<Eigen::VectorXd> newton_iterates;
    std::vector<Eigen::VectorXd> descent_iterates;

    const expected<Eigen::VectorXd, refusal> by_newton  = references.newton(never, Eigen::Vector2d(1.0, 1.0), 7u, newton_iterates);
    const expected<Eigen::VectorXd, refusal> by_descent = references.gradient_descent(never, Eigen::Vector2d(1.0, 1.0), 0.05, 7u, descent_iterates);

    REQUIRE_FALSE(by_newton);
    REQUIRE_FALSE(by_descent);
    CHECK(by_newton.error() == refusal::no_solution);
    CHECK(by_descent.error() == refusal::no_solution);
    CHECK(newton_iterates.size() == 8u);
    CHECK(descent_iterates.size() == 8u);
}

TEST_CASE("a gradient refusal is answered as it came by either minimizer with the iterates reached kept")
{
    for(const refusal kind : {refusal::degenerate, refusal::unsupported_input})
        check_gradient_refusal(kind);
}

TEST_CASE("a hessian refusal is answered as it came by Newton with the seed kept")
{
    for(const refusal kind : {refusal::degenerate, refusal::unsupported_input})
    {
        problem refusing = elongated_bowl();
        refusing.hessian = [kind](const Eigen::VectorXd &) -> expected<Eigen::MatrixXd, refusal> { return praxis::unexpected(kind); };
        std::vector<Eigen::VectorXd> iterates;

        const expected<Eigen::VectorXd, refusal> answered = references.newton(refusing, Eigen::Vector2d(1.0, 1.0), 50u, iterates);

        REQUIRE_FALSE(answered);
        CHECK(answered.error() == kind);
        CHECK(iterates.size() == 1u);
    }
}

TEST_CASE("a gradient that is not the seed's length is refused degenerate by either minimizer with the seed kept")
{
    for(const Eigen::Index length : {Eigen::Index{1}, Eigen::Index{3}})
    {
        problem misshapen  = elongated_bowl();
        misshapen.gradient = [length](const Eigen::VectorXd &) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd::Ones(length); };
        std::vector<Eigen::VectorXd> newton_iterates;
        std::vector<Eigen::VectorXd> descent_iterates;

        const expected<Eigen::VectorXd, refusal> by_newton  = references.newton(misshapen, Eigen::Vector2d(1.0, 1.0), 50u, newton_iterates);
        const expected<Eigen::VectorXd, refusal> by_descent = references.gradient_descent(misshapen, Eigen::Vector2d(1.0, 1.0), 0.05, 50u, descent_iterates);

        REQUIRE_FALSE(by_newton);
        REQUIRE_FALSE(by_descent);
        CHECK(by_newton.error() == refusal::degenerate);
        CHECK(by_descent.error() == refusal::degenerate);
        CHECK(newton_iterates.size() == 1u);
        CHECK(descent_iterates.size() == 1u);
    }
}

TEST_CASE("a hessian that is not square in the seed's length is refused degenerate by Newton with the seed kept")
{
    for(const Eigen::MatrixXd &shaped : {Eigen::MatrixXd(Eigen::MatrixXd::Identity(1, 1)), Eigen::MatrixXd(Eigen::MatrixXd::Identity(2, 3)),
                                         Eigen::MatrixXd(Eigen::MatrixXd::Identity(3, 2)), Eigen::MatrixXd(Eigen::MatrixXd::Identity(3, 3))})
    {
        problem misshapen = elongated_bowl();
        misshapen.hessian = [shaped](const Eigen::VectorXd &) -> expected<Eigen::MatrixXd, refusal> { return shaped; };
        std::vector<Eigen::VectorXd> iterates;

        const expected<Eigen::VectorXd, refusal> answered = references.newton(misshapen, Eigen::Vector2d(1.0, 1.0), 50u, iterates);

        REQUIRE_FALSE(answered);
        CHECK(answered.error() == refusal::degenerate);
        CHECK(iterates.size() == 1u);
    }
}

TEST_CASE("gradient descent refuses a step length that is not finite and positive before appending anything")
{
    for(const double step_length : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
    {
        std::vector<Eigen::VectorXd> iterates;

        const expected<Eigen::VectorXd, refusal> answered = references.gradient_descent(elongated_bowl(), Eigen::Vector2d(1.0, 1.0), step_length, 50u, iterates);

        REQUIRE_FALSE(answered);
        CHECK(answered.error() == refusal::unsupported_input);
        CHECK(iterates.empty());
    }
}

TEST_CASE("a minimizer refuses a problem leaving a member it reads empty before appending anything")
{
    problem without_gradient      = elongated_bowl();
    without_gradient.gradient     = {};
    problem without_stationary    = elongated_bowl();
    without_stationary.stationary = {};
    problem without_hessian       = elongated_bowl();
    without_hessian.hessian       = {};
    std::vector<Eigen::VectorXd> iterates;

    for(const problem &incomplete : {without_gradient, without_stationary, without_hessian})
    {
        const expected<Eigen::VectorXd, refusal> by_newton = references.newton(incomplete, Eigen::Vector2d(1.0, 1.0), 50u, iterates);
        REQUIRE_FALSE(by_newton);
        CHECK(by_newton.error() == refusal::unsupported_input);
    }
    for(const problem &incomplete : {without_gradient, without_stationary})
    {
        const expected<Eigen::VectorXd, refusal> by_descent = references.gradient_descent(incomplete, Eigen::Vector2d(1.0, 1.0), 0.05, 50u, iterates);
        REQUIRE_FALSE(by_descent);
        CHECK(by_descent.error() == refusal::unsupported_input);
    }
    CHECK(iterates.empty());
}
