#include "composed_curve.h"

#include "panel_keys.h"
#include "imgui_frame.h"

#include "praxis/optimization/solve_window.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr std::size_t seed_row  = 0;
constexpr std::size_t limit_row = 1;
constexpr std::size_t run_row   = 2;
constexpr std::size_t step_row  = 3;

double newton_from(double x)
{
    return x - cubic_value(x) / cubic_slope(x);
}

// How often the registered curve's value and its slope were asked for.
struct asked_count
{
    std::size_t valued;
    std::size_t sloped;
};

registered_curve counting(const std::shared_ptr<asked_count> &into)
{
    return registered_curve(
            "cubic",
            [into](double x)
            {
                ++into->valued;
                return cubic_value(x);
            },
            [into](double x)
            {
                ++into->sloped;
                return cubic_slope(x);
            });
}

struct solving_stage
{
    scheduler::scheduler loop;
    staged built;
    std::shared_ptr<solve_window> panel;
    opened taken;

    explicit solving_stage(const scenario_settings &settings = opening(), registered_curve curve = cubic())
            : loop(scheduler::inline_workers)
            , built(loop)
            , panel()
            , taken()
    {
        const scenario_window_composer windows = [this](const scenario_window_inputs &offered)
        {
            panel = std::make_shared<solve_window>("Solve", offered);

            return std::vector<std::shared_ptr<scene::imgui_window>>{panel};
        };
        taken = open_composing(built.site, {std::move(curve)}, baseline(), settings, windows);
        REQUIRE(settled(loop, taken) != nullptr);
    }

    drawing draw() const
    {
        return [shown = panel] { shown->render(); };
    }

    std::shared_ptr<const scenario_snapshot> drained()
    {
        REQUIRE(loop.drain().has_value());

        return taken.inputs->seen.read();
    }
};

// The step below the limit field lands on whichever of Run and Step once lies nearer its middle, so
// each is reached by a step sideways along their row.
void press_run(tests::imgui_frame &frames, const drawing &draw)
{
    stand_below_top(frames, draw, run_row);
    tap(frames, draw, ImGuiKey_LeftArrow);
    tap(frames, draw, ImGuiKey_Space);
}

void press_step_once(tests::imgui_frame &frames, const drawing &draw)
{
    stand_below_top(frames, draw, run_row);
    tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_Space);
}

// Two steps from `seed`, the newest selected, and one table row for each iterate.
void check_two_steps_from(double seed, const scenario_snapshot &stepped)
{
    REQUIRE(stepped.run.iterates.size() == 3u);
    CHECK(stepped.run.iterates[0][0] == seed);
    CHECK(stepped.run.iterates[1][0] == newton_from(seed));
    CHECK(stepped.run.iterates[2][0] == newton_from(newton_from(seed)));
    CHECK((stepped.stepped && stepped.step == 1u));
    const std::vector<iterate_row> rows = iterate_rows(stepped.run);
    REQUIRE(rows.size() == 3u);
    CHECK((rows[0].k == 0u && rows[1].k == 1u && rows[2].k == 2u));
}

}

TEST_CASE("run and step once re-run the solver from the seed", "[optimization][solve]")
{
    solving_stage stage;
    tests::imgui_frame frames;
    type_into(frames, stage.draw(), seed_row, "2");

    press_step_once(frames, stage.draw());
    static_cast<void>(stage.drained());
    press_step_once(frames, stage.draw());
    const std::shared_ptr<const scenario_snapshot> stepped = stage.drained();

    check_two_steps_from(2.0, *stepped);

    press_run(frames, stage.draw());
    const std::shared_ptr<const scenario_snapshot> ran = stage.drained();

    CHECK(ran->run.iterates.front()[0] == 2.0);
    CHECK_FALSE(ran->run.refused.has_value());
    CHECK(std::abs(cubic_value(ran->run.iterates.back()[0])) < 1.0e-10);
}

TEST_CASE("the step slider walks the run forward and back without solving again", "[optimization][solve]")
{
    const auto asked = std::make_shared<asked_count>(asked_count{0u, 0u});
    solving_stage stage(opening(), counting(asked));
    const std::shared_ptr<const scenario_snapshot> first = stage.taken.inputs->seen.read();
    REQUIRE(first->run.iterates.size() >= 4u);
    REQUIRE(first->step == 0u);
    const asked_count before = *asked;

    tests::imgui_frame frames;
    tweak_from(frames, stage.draw(), step_row, 2);
    const std::shared_ptr<const scenario_snapshot> forward = stage.drained();
    tap(frames, stage.draw(), ImGuiKey_LeftArrow);
    const std::shared_ptr<const scenario_snapshot> back = stage.drained();

    CHECK(forward->step == 2u);
    CHECK(back->step == 1u);
    CHECK(back->run.iterates == first->run.iterates);
    CHECK(asked->valued == before.valued);
    CHECK(asked->sloped == before.sloped);
}

TEST_CASE("a limit edited in the window is the one the next run stops at", "[optimization][solve]")
{
    solving_stage stage;
    tests::imgui_frame frames;
    type_into(frames, stage.draw(), limit_row, "2");
    press_run(frames, stage.draw());
    const std::shared_ptr<const scenario_snapshot> ran = stage.drained();

    CHECK(ran->run.iterates.size() == 3u);
    CHECK_FALSE(ran->stepped);
    CHECK(ran->run.refused == refusal::no_solution);
    CHECK(run_status(*ran) == "Refused: no solution. Steps taken: 2.");
}

TEST_CASE("step once stops at the iteration limit, and a step past the run is held to its last", "[optimization][solve]")
{
    solving_stage stage;
    tests::imgui_frame frames;
    type_into(frames, stage.draw(), limit_row, "1");
    press_step_once(frames, stage.draw());
    const std::shared_ptr<const scenario_snapshot> capped = stage.drained();

    CHECK(capped->run.iterates.size() == 2u);
    CHECK_FALSE(capped->stepped);
    CHECK(capped->run.refused == refusal::no_solution);

    command(stage.taken.inputs->scenario, [](scenario_state &state) { state.select_step(9u); });
    CHECK(stage.drained()->step == 0u);
}
