#include "praxis/optimization/problem.h"
#include "praxis/optimization/solver.h"
#include "praxis/optimization/projected/solver.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <cmath>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>

using namespace praxis;
using namespace praxis::optimization;

namespace {

Eigen::VectorXd point(double x)
{
    return Eigen::VectorXd::Constant(1, x);
}

Eigen::VectorXd point(double x, double y)
{
    return Eigen::Vector2d(x, y);
}

bool held_by(const box &bounds, const Eigen::VectorXd &x)
{
    return (x.array() >= bounds.lower.array()).all() && (x.array() <= bounds.upper.array()).all();
}

// g(x) = x^2 - 4 in the book's sign: the error is -g(x) and the Jacobian is dg/dx.
problem root_of_four(std::optional<box> bounds)
{
    return problem{
            .error     = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal> { return point(4.0 - x[0] * x[0]); },
            .jacobian  = [](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal> { return Eigen::MatrixXd::Constant(1, 1, 2.0 * x[0]); },
            .converged = [](const Eigen::VectorXd &e) -> bool { return e.cwiseAbs().maxCoeff() < 1.0e-12; },
            .bounds    = std::move(bounds),
    };
}

Eigen::VectorXd circle_error(const Eigen::VectorXd &x)
{
    return point(1.0 - x.squaredNorm());
}

Eigen::MatrixXd circle_jacobian(const Eigen::VectorXd &x)
{
    return Eigen::MatrixXd(2.0 * x.transpose());
}

// f(x, y) = x^2 + y^2 - 1: one equation in two unknowns, so the zero set is the unit circle.
problem unit_circle(std::optional<box> bounds)
{
    return problem{
            .error     = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal> { return circle_error(x); },
            .jacobian  = [](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal> { return circle_jacobian(x); },
            .converged = [](const Eigen::VectorXd &e) -> bool { return e.cwiseAbs().maxCoeff() < 1.0e-12; },
            .bounds    = std::move(bounds),
    };
}

}

TEST_CASE("a step the box returns to the iterate it left is refused with every iterate kept inside the box")
{
    const box bounds{point(0.0), point(1.0)};
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> root = projected::solve(root_of_four(bounds), point(0.5), 50u, iterates);

    REQUIRE_FALSE(root);
    CHECK(root.error() == refusal::no_solution);
    REQUIRE(iterates.size() >= 2u);
    CHECK(iterates.front() == point(0.5));
    CHECK(iterates.back() == point(1.0));
    CHECK(iterates.size() < 50u);
    for(std::size_t k = 1; k < iterates.size(); ++k)
    {
        CHECK(held_by(bounds, iterates[k]));
        CHECK(iterates[k] != iterates[k - 1]);
    }
}

TEST_CASE("a root inside the box is reached, and without a box the solve is unprojected")
{
    const box bounds{point(0.0), point(3.0)};
    std::vector<Eigen::VectorXd> boxed;
    std::vector<Eigen::VectorXd> free;

    const expected<Eigen::VectorXd, refusal> inside    = projected::solve(root_of_four(bounds), point(0.5), 50u, boxed);
    const expected<Eigen::VectorXd, refusal> unbounded = projected::solve(root_of_four(std::nullopt), point(0.5), 50u, free);

    REQUIRE(inside);
    CHECK(std::abs((*inside)[0] - 2.0) < 1.0e-12);
    CHECK(boxed.back() == *inside);
    REQUIRE(unbounded);
    CHECK(std::abs((*unbounded)[0] - 2.0) < 1.0e-12);
    CHECK(free.front() == point(0.5));
}

TEST_CASE("a seed outside the box is recorded as handed and the first step lands inside it")
{
    const box bounds{point(0.0), point(3.0)};
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> root = projected::solve(root_of_four(bounds), point(5.0), 50u, iterates);

    REQUIRE(root);
    REQUIRE(iterates.size() >= 2u);
    CHECK(iterates[0] == point(5.0));
    CHECK(held_by(bounds, iterates[1]));
}

TEST_CASE("the first step is the levenberg-marquardt step damped by half the squared error and the bias")
{
    const Eigen::VectorXd x0 = point(1.5, 1.5);
    std::vector<Eigen::VectorXd> iterates;
    static_cast<void>(projected::solve(unit_circle(std::nullopt), x0, 1u, iterates));

    const Eigen::MatrixXd j            = circle_jacobian(x0);
    const Eigen::VectorXd e            = circle_error(x0);
    const Eigen::MatrixXd coefficients = j.transpose() * j + (0.5 * e.dot(e) + 1.0e-8) * Eigen::MatrixXd::Identity(2, 2);
    const Eigen::VectorXd step         = coefficients.llt().solve(j.transpose() * e);

    REQUIRE(iterates.size() == 2u);
    CHECK(iterates[1] == Eigen::VectorXd(x0 + step));
}

TEST_CASE("an underdetermined problem is answered by a point on its zero set inside the box")
{
    const box bounds{point(0.8, 0.0), point(2.0, 2.0)};
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> root = projected::solve(unit_circle(bounds), point(1.5, 1.5), 100u, iterates);

    REQUIRE(root);
    CHECK(std::abs(root->squaredNorm() - 1.0) < 1.0e-12);
    CHECK(held_by(bounds, *root));
    for(std::size_t k = 1; k < iterates.size(); ++k)
        CHECK(held_by(bounds, iterates[k]));
}

TEST_CASE("an error refused past the seed is passed on with the iterates kept")
{
    problem refusing = root_of_four(std::nullopt);
    refusing.error   = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal>
    {
        if(x[0] != 0.5)
            return praxis::unexpected(refusal::degenerate);
        return point(4.0 - x[0] * x[0]);
    };
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> root = projected::solve(refusing, point(0.5), 50u, iterates);

    REQUIRE_FALSE(root);
    CHECK(root.error() == refusal::degenerate);
    REQUIRE(iterates.size() == 2u);
    CHECK(iterates.front() == point(0.5));
}

TEST_CASE("an empty member or a box that does not fit the seed is refused before anything is recorded")
{
    problem without_jacobian  = root_of_four(std::nullopt);
    without_jacobian.jacobian = nullptr;

    for(const problem &p :
        {without_jacobian, root_of_four(box{point(0.0, 0.0), point(3.0, 3.0)}), root_of_four(box{point(0.0), point(3.0, 3.0)}), root_of_four(box{point(1.0), point(0.0)})})
    {
        std::vector<Eigen::VectorXd> iterates;
        const expected<Eigen::VectorXd, refusal> root = projected::solve(p, point(0.5), 50u, iterates);
        REQUIRE_FALSE(root);
        CHECK(root.error() == refusal::unsupported_input);
        CHECK(iterates.empty());
    }
}

TEST_CASE("a limit of zero judges the seed and takes no step")
{
    std::vector<Eigen::VectorXd> unconverged;
    std::vector<Eigen::VectorXd> converged;

    const expected<Eigen::VectorXd, refusal> refused  = projected::solve(root_of_four(std::nullopt), point(0.5), 0u, unconverged);
    const expected<Eigen::VectorXd, refusal> answered = projected::solve(root_of_four(std::nullopt), point(2.0), 0u, converged);

    REQUIRE_FALSE(refused);
    CHECK(refused.error() == refusal::no_solution);
    CHECK(unconverged.size() == 1u);
    REQUIRE(answered);
    CHECK(*answered == point(2.0));
    CHECK(converged.size() == 1u);
}
