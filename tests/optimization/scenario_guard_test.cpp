#include "composed_curve.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/solve_window.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <memory>
#include <vector>
#include <cstdint>

using namespace praxis;
using namespace praxis::fixture;

namespace {

expected<Eigen::VectorXd, refusal> answering_the_seed(const problem &, const Eigen::VectorXd &x0, std::uint32_t, std::vector<Eigen::VectorXd> &iterates)
{
    iterates.push_back(x0);

    return x0;
}

}

TEST_CASE("a refusal other than the steps running out is shown after a single step", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const opened taken = open_with(built.site, {cubic()}, capabilities{}, opening());
    REQUIRE(settled(loop, taken) != nullptr);

    command(taken.inputs->scenario, [](scenario_state &state) { state.step_once(Eigen::VectorXd::Constant(1, 3.0)); });
    REQUIRE(loop.drain().has_value());
    const std::shared_ptr<const scenario_snapshot> stepped = taken.inputs->seen.read();

    REQUIRE(stepped->stepped);
    CHECK(run_status(*stepped) == "Refused: not implemented. Steps taken: 0.");

    taken.composed->tear_down();
}

TEST_CASE("an answer at which the problem has not converged is not shown as converged", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const opened taken                                 = open_with(built.site, {cubic()}, capabilities{.solver = {.solve = &answering_the_seed}}, opening());
    const std::shared_ptr<const scenario_snapshot> ran = settled(loop, taken);

    REQUIRE(ran->run.answer.has_value());
    CHECK(*ran->run.answer == Eigen::VectorXd::Constant(1, 3.0));
    CHECK(ran->run.refused == refusal::no_solution);
    CHECK(run_status(*ran) == "Answered where the problem has not converged. Steps taken: 0.");

    taken.composed->tear_down();
}
