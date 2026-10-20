#include "composed_curve.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "captured_log.h"

#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/solve_window.h"
#include "praxis/optimization/function_window.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <span>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr std::size_t function_row = 1;
constexpr std::size_t lower_row    = 2;
constexpr std::size_t upper_row    = 3;

double cosine_less_x(double x)
{
    return std::cos(x) - x;
}

double cosine_less_x_slope(double x)
{
    return -std::sin(x) - 1.0;
}

double shifted(double x)
{
    return x - 1.0;
}

double unit_slope(double)
{
    return 1.0;
}

std::vector<registered_curve> three_curves()
{
    return {cubic(), registered_curve("cos(x) - x", &cosine_less_x, &cosine_less_x_slope), registered_curve("x - 1", &shifted, &unit_slope)};
}

struct function_stage
{
    scheduler::scheduler loop;
    staged built;
    std::shared_ptr<function_window> panel;
    opened taken;

    function_stage()
            : loop(scheduler::inline_workers)
            , built(loop)
            , panel()
            , taken()
    {
        const scenario_window_composer windows = [this](const scenario_window_inputs &offered)
        {
            panel = std::make_shared<function_window>("Function", offered);

            return std::vector<std::shared_ptr<scene::imgui_window>>{curve_graph(offered, "Graph"), panel};
        };
        taken = open_composing(built.site, three_curves(), baseline(), opening(), windows);
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

// The list opens on its first entry whichever is selected, so the entry taken is counted from the top.
void take_entry(tests::imgui_frame &frames, const drawing &draw, std::size_t entry)
{
    stand_below_top(frames, draw, function_row);
    tap(frames, draw, ImGuiKey_Space);
    REQUIRE(popup_open());
    for(std::size_t step = 0; step < entry; ++step)
        tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_Space);
}

}

TEST_CASE("the function window's source selector offers one entry, the registered functions", "[optimization][function]")
{
    const std::span<const char *const> labels = function_source_labels();

    REQUIRE(labels.size() == 1u);
    CHECK(std::string(labels[0]) == "Registered");
    CHECK(static_cast<std::size_t>(function_source::registered) == 0u);
}

TEST_CASE("the function list walks the registered functions in the order handed and each re-runs from the seed", "[optimization][function]")
{
    function_stage stage;
    REQUIRE(stage.taken.inputs->registered == std::vector<std::string>{"cubic", "cos(x) - x", "x - 1"});

    for(std::size_t next = 1u; next < 3u; ++next)
    {
        tests::imgui_frame frames;
        take_entry(frames, stage.draw(), next);
        const std::shared_ptr<const scenario_snapshot> chosen = stage.drained();

        CHECK(chosen->selected == next);
        CHECK(chosen->run.iterates.front()[0] == 3.0);
        CHECK_FALSE(chosen->run.refused.has_value());
    }
}

TEST_CASE("an edited range moves the samples and the graph's limits, and a minimum not below the maximum is not posted", "[optimization][function]")
{
    function_stage stage;
    tests::imgui_frame frames;

    type_into(frames, stage.draw(), lower_row, "-2");
    const std::shared_ptr<const scenario_snapshot> narrowed = stage.drained();
    CHECK((narrowed->ranges.front().lower == -2.0 && narrowed->ranges.front().upper == 4.0));
    CHECK(narrowed->abscissa.front() == -2.0);
    const scene::plot_reading drawn = curve_reading(stage.taken.inputs->seen);
    REQUIRE(drawn.frames.size() == 1u);
    CHECK(drawn.frames.front().abscissa_limits->lower == -2.0);

    const tests::captured_log captured;
    type_into(frames, stage.draw(), upper_row, "-3");
    CHECK(stage.drained() == narrowed);
    CHECK(captured.text().empty());
}

TEST_CASE("a composition whose composer leaves the function window out opens and runs", "[optimization][function]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const scenario_window_composer solve_only = [](const scenario_window_inputs &offered)
    { return std::vector<std::shared_ptr<scene::imgui_window>>{std::make_shared<solve_window>("Solve", offered)}; };

    const opened taken                                 = open_composing(built.site, three_curves(), baseline(), opening(), solve_only);
    const std::shared_ptr<const scenario_snapshot> ran = settled(loop, taken);

    REQUIRE(ran != nullptr);
    REQUIRE(taken.composed->windows.size() == 1u);
    CHECK_FALSE(ran->run.refused.has_value());
    CHECK(std::abs(cubic_value(ran->run.iterates.back()[0])) < 1.0e-10);
}
