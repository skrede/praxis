#include "book_solvers.h"

#include "praxis/manipulator/evaluation.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/optimization/solver.h"
#include "praxis/optimization/problem.h"
#include "praxis/optimization/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <vector>
#include <cstdint>

using namespace praxis;
using namespace praxis::evaluation;
using namespace praxis::tests::book;

namespace {

slot_report judged(const optimization::capabilities &first, const optimization::capabilities &second, std::uint64_t seed)
{
    const std::array<evaluation_view, 1> views = manipulator::solver_evaluation_views(first, second);

    return evaluate(std::span(views), seed, default_cases_per_slot).slots.at(0);
}

// Newton-Raphson with the step's sign reversed, x_{k+1} = x_k - J^+(x_k) e(x_k).
expected<Eigen::VectorXd, refusal> stepping_away(const optimization::problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    Eigen::VectorXd x = x0;
    iterates.push_back(x);
    expected<Eigen::VectorXd, refusal> e = p.error(x);
    for(std::uint32_t i = 0; e && !p.converged(*e) && i < max_iterations; ++i)
    {
        const expected<Eigen::MatrixXd, refusal> j = p.jacobian(x);
        if(!j)
            return praxis::unexpected(j.error());
        x -= solver_step_along<solver_step::cod>(*j, *e);
        iterates.push_back(x);
        e = p.error(x);
    }
    if(!e)
        return praxis::unexpected(e.error());
    if(!p.converged(*e))
        return praxis::unexpected(refusal::no_solution);
    return x;
}

}

TEST_CASE("the adapter route judges the reference solver against itself as agreeing or both refusing")
{
    const optimization::capabilities reference = optimization::baseline();
    const slot_report row                      = judged(reference, reference, default_seed);

    REQUIRE(row.extension == "manipulator");
    REQUIRE(row.slot == "solver.solve");
    REQUIRE(row.cases == default_cases_per_slot);
    CHECK(row.outcomes.agreed + row.outcomes.both_refused == row.cases);
    CHECK(row.outcomes.agreed > 0u);
    CHECK(row.verdict == agreement::agreed);
}

TEST_CASE("every literal solver driven through the inverse kinematics adapters passes the adapter route")
{
    const optimization::capabilities reference = optimization::baseline();
    for(const literal_solver &form : literal_solvers)
    {
        const optimization::capabilities literal{.solver = optimization::solver_ops{.solve = form.solve}};
        for(std::uint64_t seed = default_seed; seed <= default_seed + 40u; ++seed)
        {
            const slot_report row = judged(literal, reference, seed);

            INFO(solver_label(form) << ", seed " << seed);
            REQUIRE(row.slot == "solver.solve");
            REQUIRE(row.cases == default_cases_per_slot);
            CHECK(row.outcomes.differed + row.outcomes.refused_differently + row.outcomes.unusable == 0u);
            CHECK(row.outcomes.agreed > 0u);
        }
    }
}

TEST_CASE("a solver stepping away from the root fails the adapter route")
{
    const optimization::capabilities reference = optimization::baseline();
    const optimization::capabilities away{.solver = optimization::solver_ops{.solve = &stepping_away}};
    const slot_report row = judged(away, reference, default_seed);

    REQUIRE(row.slot == "solver.solve");
    CHECK(row.verdict != agreement::agreed);
}
