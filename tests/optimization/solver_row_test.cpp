#include "book_solvers.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/evaluation.h"
#include "praxis/optimization/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/QR>
#include <Eigen/Core>

#include <array>
#include <vector>
#include <cstddef>
#include <cstdint>

using namespace praxis;
using namespace praxis::optimization;
using namespace praxis::evaluation;
using namespace praxis::tests::book;

namespace {

// The book's iteration with the step's sign reversed, x_{k+1} = x_k - J^+ e.
expected<Eigen::VectorXd, refusal> stepping_away(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    Eigen::VectorXd x = x0;
    iterates.push_back(x);
    expected<Eigen::VectorXd, refusal> e = p.error(x);
    for(std::uint32_t i = 0; e && !p.converged(*e) && i < max_iterations; ++i)
    {
        const expected<Eigen::MatrixXd, refusal> j = p.jacobian(x);
        if(!j)
            return praxis::unexpected(j.error());
        x -= Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>(*j).solve(*e);
        iterates.push_back(x);
        e = p.error(x);
    }
    if(!e)
        return praxis::unexpected(e.error());
    if(!p.converged(*e))
        return praxis::unexpected(refusal::no_solution);
    return x;
}

capabilities bound_to(solve_slot solve)
{
    capabilities bound = baseline();
    bound.solver.solve = solve;
    return bound;
}

slot_report judged(const capabilities &first, const capabilities &second, std::uint64_t seed)
{
    const std::array<evaluation_view, 1> views = evaluation_views(first, second);

    return evaluate(views, seed, default_cases_per_slot).slots.at(0);
}

}

TEST_CASE("the solver row judges the reference against itself as agreeing or both refusing")
{
    const capabilities reference = baseline();
    const slot_report row        = judged(reference, reference, default_seed);

    REQUIRE(row.extension == "optimization");
    REQUIRE(row.slot == "solver.solve");
    REQUIRE(row.cases == default_cases_per_slot);
    CHECK(row.verdict == agreement::agreed);
    CHECK(row.outcomes.agreed + row.outcomes.both_refused == row.cases);
    CHECK(row.outcomes.agreed > 0u);
}

TEST_CASE("every literal Newton-Raphson form passes the solver row")
{
    const capabilities reference = baseline();
    std::vector<std::uint64_t> seeds{default_seed};
    for(std::uint64_t seed = 1; seed <= 40u; ++seed)
        seeds.push_back(seed);

    for(const literal_solver &form : literal_solvers)
    {
        const capabilities literal = bound_to(form.solve);
        for(const std::uint64_t seed : seeds)
        {
            const slot_report row = judged(literal, reference, seed);

            INFO(solver_label(form) << ", seed " << seed);
            REQUIRE(row.slot == "solver.solve");
            REQUIRE(row.cases == default_cases_per_slot);
            CHECK(row.outcomes.differed + row.outcomes.refused_differently + row.outcomes.unusable == 0u);
            CHECK(row.outcomes.agreed > 0u);
            CHECK(row.verdict == agreement::agreed);
        }
    }
}

TEST_CASE("a solve stepping against the book's sign fails the solver row")
{
    const capabilities reference = baseline();
    const capabilities away      = bound_to(&stepping_away);
    const slot_report row        = judged(away, reference, default_seed);

    INFO("agreed " << row.outcomes.agreed << ", both refused " << row.outcomes.both_refused << ", one refused " << row.outcomes.one_refused << ", differed " << row.outcomes.differed
                   << ", refused differently " << row.outcomes.refused_differently);
    REQUIRE(row.slot == "solver.solve");
    REQUIRE(row.cases == default_cases_per_slot);
    CHECK(row.outcomes.agreed == 0u);
    CHECK(row.outcomes.one_refused > 0u);
    CHECK(row.verdict != agreement::agreed);
}
