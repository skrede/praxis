#include "judged_row.h"
#include "book_minimizers.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/evaluation.h"
#include "praxis/optimization/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/QR>
#include <Eigen/Core>

#include <vector>
#include <cstdint>
#include <string_view>

using namespace praxis;
using namespace praxis::optimization;
using namespace praxis::evaluation;
using namespace praxis::tests::book;
using praxis::tests::judged;

namespace {

constexpr std::string_view newton_row  = "minimizer.newton";
constexpr std::string_view descent_row = "minimizer.gradient_descent";

// Iterates x_{k+1} = x_k + step(x_k, g_k), the sign the book's methods negate.
template<typename Step>
expected<Eigen::VectorXd, refusal> stepping_uphill(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates, Step step)
{
    Eigen::VectorXd x = x0;
    iterates.push_back(x);
    expected<Eigen::VectorXd, refusal> g = p.gradient(x);
    for(std::uint32_t i = 0; g && !p.stationary(*g) && i < max_iterations; ++i)
    {
        const expected<Eigen::VectorXd, refusal> stepped = step(x, *g);
        if(!stepped)
            return praxis::unexpected(stepped.error());
        x += *stepped;
        iterates.push_back(x);
        g = p.gradient(x);
    }
    if(!g)
        return praxis::unexpected(g.error());
    if(!p.stationary(*g))
        return praxis::unexpected(refusal::no_solution);
    return x;
}

expected<Eigen::VectorXd, refusal> newton_reversed(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    return stepping_uphill(p, x0, max_iterations, iterates,
                           [&p](const Eigen::VectorXd &x, const Eigen::VectorXd &g) -> expected<Eigen::VectorXd, refusal>
                           {
                               const expected<Eigen::MatrixXd, refusal> h = p.hessian(x);
                               if(!h)
                                   return praxis::unexpected(h.error());
                               return Eigen::VectorXd(Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>(*h).solve(g));
                           });
}

expected<Eigen::VectorXd, refusal> ascent(const problem &p, const Eigen::VectorXd &x0, double step_length, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    return stepping_uphill(p, x0, max_iterations, iterates,
                           [step_length](const Eigen::VectorXd &, const Eigen::VectorXd &g) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd(step_length * g); });
}

capabilities bound_to(newton_slot newton, descent_slot gradient_descent)
{
    capabilities bound               = baseline();
    bound.minimizer.newton           = newton;
    bound.minimizer.gradient_descent = gradient_descent;
    return bound;
}

std::vector<std::uint64_t> judged_seeds()
{
    std::vector<std::uint64_t> seeds{default_seed};
    for(std::uint64_t seed = 1; seed <= 40u; ++seed)
        seeds.push_back(seed);
    return seeds;
}

void require_passed(const slot_report &row)
{
    REQUIRE(row.cases == default_cases_per_slot);
    CHECK(row.outcomes.differed + row.outcomes.refused_differently + row.outcomes.unusable == 0u);
    CHECK(row.outcomes.agreed > 0u);
    CHECK(row.verdict == agreement::agreed);
}

}

TEST_CASE("the minimizer rows judge the references against themselves as agreeing or both refusing")
{
    const capabilities reference = baseline();
    for(const std::string_view name : {newton_row, descent_row})
    {
        const slot_report row = judged(reference, reference, name, default_seed);

        INFO(name);
        REQUIRE(row.extension == "optimization");
        require_passed(row);
        CHECK(row.outcomes.agreed + row.outcomes.both_refused == row.cases);
    }
}

TEST_CASE("every literal Newton form passes the Newton row")
{
    const capabilities reference = baseline();
    for(const literal_newton &form : literal_newtons)
        for(const std::uint64_t seed : judged_seeds())
        {
            INFO(newton_label(form) << ", seed " << seed);
            require_passed(judged(bound_to(form.newton, reference.minimizer.gradient_descent), reference, newton_row, seed));
        }
}

TEST_CASE("every literal steepest-descent form passes the descent row")
{
    const capabilities reference = baseline();
    for(const literal_descent &form : literal_descents)
        for(const std::uint64_t seed : judged_seeds())
        {
            INFO(descent_label(form) << ", seed " << seed);
            require_passed(judged(bound_to(reference.minimizer.newton, form.gradient_descent), reference, descent_row, seed));
        }
}

TEST_CASE("a step against the book's sign fails its minimizer row")
{
    const capabilities reference = baseline();
    const capabilities uphill    = bound_to(&newton_reversed, &ascent);
    for(const std::string_view name : {newton_row, descent_row})
    {
        const slot_report row = judged(uphill, reference, name, default_seed);

        INFO(name << ": agreed " << row.outcomes.agreed << ", both refused " << row.outcomes.both_refused << ", one refused " << row.outcomes.one_refused << ", differed "
                  << row.outcomes.differed << ", refused differently " << row.outcomes.refused_differently);
        REQUIRE(row.cases == default_cases_per_slot);
        CHECK(row.outcomes.agreed == 0u);
        CHECK(row.outcomes.both_refused + row.outcomes.differed + row.outcomes.refused_differently + row.outcomes.unusable == 0u);
        CHECK(row.outcomes.one_refused == row.cases);
        CHECK(row.verdict == agreement::one_refused);
    }
}
