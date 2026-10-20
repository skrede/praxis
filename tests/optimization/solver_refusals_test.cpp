#include "praxis/optimization/problem.h"
#include "praxis/optimization/solver.h"
#include "praxis/optimization/baseline/solver.h"
#include "praxis/optimization/projected/solver.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <vector>
#include <cstdint>

using namespace praxis;
using namespace praxis::optimization;

namespace {

using solving = expected<Eigen::VectorXd, refusal> (*)(const problem &, const Eigen::VectorXd &, std::uint32_t, std::vector<Eigen::VectorXd> &);

// f(x, y) = x^2 + y^2 - 1, one equation in two unknowns, whose Jacobian is answered as `shaped`.
problem circle_answering(Eigen::MatrixXd shaped)
{
    return problem{
            .error     = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd::Constant(1, 1.0 - x.squaredNorm()); },
            .jacobian  = [shaped](const Eigen::VectorXd &) -> expected<Eigen::MatrixXd, refusal> { return shaped; },
            .converged = [](const Eigen::VectorXd &e) -> bool { return e.cwiseAbs().maxCoeff() < 1.0e-12; },
    };
}

}

TEST_CASE("a jacobian that is not the error's length by the seed's is refused degenerate by either solve with the seed kept")
{
    const Eigen::VectorXd x0 = Eigen::Vector2d(0.3, 0.2);

    for(const solving solve_with : {static_cast<solving>(&solve), static_cast<solving>(&projected::solve)})
        for(const Eigen::MatrixXd &shaped : {Eigen::MatrixXd(Eigen::MatrixXd::Ones(1, 1)), Eigen::MatrixXd(Eigen::MatrixXd::Ones(1, 3)), Eigen::MatrixXd(Eigen::MatrixXd::Ones(2, 2))})
        {
            std::vector<Eigen::VectorXd> iterates;
            const expected<Eigen::VectorXd, refusal> root = solve_with(circle_answering(shaped), x0, 50u, iterates);

            REQUIRE_FALSE(root);
            CHECK(root.error() == refusal::degenerate);
            REQUIRE(iterates.size() == 1u);
            CHECK(iterates.front() == x0);
        }
}

TEST_CASE("a damped system whose half squared error overflows is refused degenerate rather than taken as a step of zero")
{
    const problem overflowing{
            .error     = [](const Eigen::VectorXd &) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd::Constant(1, 1.0e200); },
            .jacobian  = [](const Eigen::VectorXd &) -> expected<Eigen::MatrixXd, refusal> { return Eigen::MatrixXd::Ones(1, 1); },
            .converged = [](const Eigen::VectorXd &e) -> bool { return e.cwiseAbs().maxCoeff() < 1.0e-12; },
    };
    std::vector<Eigen::VectorXd> iterates;

    const expected<Eigen::VectorXd, refusal> root = projected::solve(overflowing, Eigen::VectorXd::Zero(1), 50u, iterates);

    REQUIRE_FALSE(root);
    CHECK(root.error() == refusal::degenerate);
    CHECK(iterates.size() == 1u);
}
