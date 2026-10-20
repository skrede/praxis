#include "composed_curve.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "captured_log.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/solve_window.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"
#include "praxis/optimization/linearization_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <limits>
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

TEST_CASE("settings opening over a range that is not drawn or from a seed that is not finite compose nothing", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const double nan = std::numeric_limits<double>::quiet_NaN();

    for(const interval &range : {interval{4.0, -4.0}, interval{1.0, 1.0}, interval{-4.0, nan}})
    {
        scenario_settings opened = opening();
        opened.ranges            = {range};
        const tests::captured_log captured;

        CHECK(open_with(built.site, {cubic()}, baseline(), opened).composed == nullptr);
        CHECK_THAT(captured.text(), Catch::Matchers::ContainsSubstring("whose minimum does not lie below its maximum"));
    }

    scenario_settings unseeded = opening();
    unseeded.seed[0]           = nan;
    const tests::captured_log captured;

    CHECK(open_with(built.site, {cubic()}, baseline(), unseeded).composed == nullptr);
    CHECK_THAT(captured.text(), Catch::Matchers::ContainsSubstring("a seed that is not finite"));
}

TEST_CASE("a settings edit through the gate that the state's own requests would refuse is logged and changes nothing", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const opened taken                                   = open_with(built.site, {cubic()}, baseline(), opening());
    const std::shared_ptr<const scenario_snapshot> first = settled(loop, taken);

    const tests::captured_log captured;
    command(taken.inputs->scenario, [](scenario_settings &settings) { settings.seed = Eigen::VectorXd::Zero(3); });
    command(taken.inputs->scenario, [](scenario_settings &settings) { settings.ranges.clear(); });
    command(taken.inputs->scenario, [](scenario_settings &settings) { settings.ranges.front() = interval{4.0, -4.0}; });
    command(taken.inputs->scenario, [](scenario_settings &settings) { settings.selected = "absent"; });
    REQUIRE(loop.drain().has_value());

    CHECK(taken.inputs->seen.read() == first);
    CHECK_THAT(captured.text(), Catch::Matchers::ContainsSubstring("a vector of 3 values"));
    CHECK_THAT(captured.text(), Catch::Matchers::ContainsSubstring("no function named 'absent' is registered"));

    command(taken.inputs->scenario, [](scenario_state &state) { state.run(); });
    REQUIRE(loop.drain().has_value());
    CHECK(taken.inputs->seen.read()->run.iterates == first->run.iterates);

    taken.composed->tear_down();
}

TEST_CASE("a point typed into the Linearization window is asked for only once its field is left", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    std::shared_ptr<linearization_window> panel;
    const scenario_window_composer composer = [&panel](const scenario_window_inputs &offered)
    {
        panel = std::make_shared<linearization_window>("Linearization", offered);

        return std::vector<std::shared_ptr<scene::imgui_window>>{panel};
    };
    const opened taken = open_composing(built.site, {cubic()}, baseline(), opening(), composer);
    REQUIRE(settled(loop, taken) != nullptr);
    const drawing draw = [panel] { panel->render(); };

    tests::imgui_frame frames;
    stand_below_top(frames, draw, 0);
    tap(frames, draw, ImGuiKey_Space);
    ImGui::GetIO().AddInputCharacter(static_cast<unsigned int>('1'));
    frames.draw_frame(draw);
    REQUIRE(loop.drain().has_value());
    CHECK(taken.inputs->seen.read()->linearization->point[0] == 0.0);

    tap(frames, draw, ImGuiKey_Enter);
    REQUIRE(loop.drain().has_value());
    CHECK(taken.inputs->seen.read()->linearization->point[0] == 1.0);

    taken.composed->tear_down();
}
