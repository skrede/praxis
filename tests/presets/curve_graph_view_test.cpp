#include "imgui_frame.h"
#include "panel_labels.h"
#include "composed_optimization.h"

#include "praxis/optimization/registered.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/store.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <implot.h>
#include <imgui_internal.h>
#include <implot_internal.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#ifndef PRAXIS_SHIPPED_OPTIMIZATION_DIR
    #define PRAXIS_SHIPPED_OPTIMIZATION_DIR ""
#endif

using namespace praxis;
using namespace praxis::fixture;

namespace {

const char *const graph_name = "Graph##curve";
const char *const solve_name = "Solve##curve";

double cubic(double x)
{
    return x * x * x - 2.0 * x - 5.0;
}

double cubic_slope(double x)
{
    return 3.0 * x * x - 2.0;
}

std::vector<optimization::registered_curve> shipped_curves()
{
    return {optimization::registered_curve("x^3 - 2x - 5", &cubic, &cubic_slope)};
}

std::shared_ptr<scene::imgui_window> window_of(const composed_optimization &composed, const char *named)
{
    for(const std::shared_ptr<scene::imgui_window> &window : composed.preset->windows)
        if(window->display_name() == named)
            return window;
    FAIL("no window is named " << named);

    return nullptr;
}

// The Solve window and the Graph at the rectangles their opening placements give them on a 1920 x
// 1080 work area at font scale 1; the Solve window comes first, so a keyboard walk focuses it.
drawing opening_layout(const composed_optimization &composed)
{
    std::vector<std::shared_ptr<scene::imgui_window>> shown{window_of(composed, solve_name), window_of(composed, graph_name)};

    return [shown]
    {
        for(const std::shared_ptr<scene::imgui_window> &window : shown)
        {
            const scene::window_rectangle at = scene::opening_rectangle(*window->opening(), ImVec2(0.f, 0.f), ImVec2(1920.f, 1080.f), 13.f);
            ImGui::SetNextWindowPos(at.position, ImGuiCond_Always);
            ImGui::SetNextWindowSize(at.size, ImGuiCond_Always);
            window->render();
        }
    };
}

const ImPlotPlot &graph_plot()
{
    ImGuiWindow *const drawn = ImGui::FindWindowByName(graph_name);
    REQUIRE(drawn != nullptr);
    const ImPlotPlot *plot = ImPlot::GetCurrentContext()->Plots.GetByKey(drawn->GetID("##plot0"));
    REQUIRE(plot != nullptr);

    return *plot;
}

ImPlotRange range_of(ImAxis axis)
{
    return graph_plot().Axes[axis].Range;
}

bool same(const ImPlotRange &one, const ImPlotRange &other)
{
    return one.Min == other.Min && one.Max == other.Max;
}

void point_at(tests::imgui_frame &frames, const drawing &draw, ImVec2 spot)
{
    ImGui::GetIO().AddMousePosEvent(spot.x, spot.y);
    frames.draw_frame(draw);
}

void left_press(tests::imgui_frame &frames, const drawing &draw, ImVec2 spot, ImVec2 drag)
{
    point_at(frames, draw, spot);
    ImGui::GetIO().AddMouseButtonEvent(ImGuiMouseButton_Left, true);
    frames.draw_frame(draw);
    point_at(frames, draw, ImVec2(spot.x + drag.x, spot.y + drag.y));
    ImGui::GetIO().AddMouseButtonEvent(ImGuiMouseButton_Left, false);
    frames.draw(draw, 3);
}

void wheel_up(tests::imgui_frame &frames, const drawing &draw)
{
    point_at(frames, draw, graph_plot().PlotRect.GetCenter());
    ImGui::GetIO().AddMouseWheelEvent(0.f, 1.f);
    frames.draw(draw, 3);
}

// The step is typed into the Solve window's slider, and the selection republished before the next
// frame reads it.
void select_step(composed_optimization &composed, tests::imgui_frame &frames, const drawing &draw, const char *step)
{
    stand_on(frames, draw, solve_name, "Step");
    tap(frames, draw, ImGuiKey_Enter);
    for(const char *at = step; *at != 0; ++at)
        ImGui::GetIO().AddInputCharacter(static_cast<unsigned int>(*at));
    frames.draw_frame(draw);
    tap(frames, draw, ImGuiKey_Enter);
    REQUIRE(composed.loop.drain().has_value());
    frames.draw(draw, 3);
}

struct opened_graph
{
    composed_optimization composed;
    drawing draw;
    tests::imgui_frame frames;

    opened_graph()
            : composed(config::resolve("curve.xml", PRAXIS_SHIPPED_OPTIMIZATION_DIR), shipped_curves(), "Root finding: a cubic")
            , draw(opening_layout(composed))
            , frames()
    {
        // A pointer's first position carries no motion, and only a moving pointer takes hovering back
        // from a keyboard walk.
        ImGui::GetIO().AddMousePosEvent(0.f, 0.f);
        frames.draw(draw, 4);
    }
};

}

TEST_CASE("the curve Graph keeps a zoom across frames and a step selection, and a double click returns it to the fitted view", "[presets][optimization]")
{
    opened_graph graph;
    CHECK(same(range_of(ImAxis_X1), ImPlotRange(-3.0, 3.0)));
    const ImPlotRange opening = range_of(ImAxis_Y1);
    select_step(graph.composed, graph.frames, graph.draw, "2");
    const ImPlotRange second = range_of(ImAxis_Y1);
    select_step(graph.composed, graph.frames, graph.draw, "0");
    REQUIRE(same(range_of(ImAxis_Y1), opening));

    wheel_up(graph.frames, graph.draw);
    const ImPlotRange across = range_of(ImAxis_X1);
    const ImPlotRange up     = range_of(ImAxis_Y1);
    REQUIRE(across.Size() < 6.0);
    select_step(graph.composed, graph.frames, graph.draw, "2");
    CHECK(same(range_of(ImAxis_X1), across));
    CHECK(same(range_of(ImAxis_Y1), up));

    for(int click = 0; click < 2; ++click)
        left_press(graph.frames, graph.draw, graph_plot().PlotRect.GetCenter(), ImVec2(0.f, 0.f));
    CHECK(same(range_of(ImAxis_X1), ImPlotRange(-3.0, 3.0)));
    CHECK(same(range_of(ImAxis_Y1), second));
    select_step(graph.composed, graph.frames, graph.draw, "0");
    CHECK(same(range_of(ImAxis_Y1), opening));
}

TEST_CASE("a left drag across the curve Graph shifts both of its ranges by the drag, and the shift stays", "[presets][optimization]")
{
    opened_graph graph;
    const ImPlotPlot &plot   = graph_plot();
    const ImPlotRange across = range_of(ImAxis_X1);
    const ImPlotRange up     = range_of(ImAxis_Y1);
    const double rightwards  = 60.0 * across.Size() / plot.PlotRect.GetWidth();
    const double downwards   = 40.0 * up.Size() / plot.PlotRect.GetHeight();

    left_press(graph.frames, graph.draw, plot.PlotRect.GetCenter(), ImVec2(60.f, 40.f));
    graph.frames.draw(graph.draw, 4);

    CHECK(std::abs(range_of(ImAxis_X1).Min - (across.Min - rightwards)) <= 1e-9 * across.Size());
    CHECK(std::abs(range_of(ImAxis_X1).Max - (across.Max - rightwards)) <= 1e-9 * across.Size());
    CHECK(std::abs(range_of(ImAxis_Y1).Min - (up.Min + downwards)) <= 1e-9 * up.Size());
    CHECK(std::abs(range_of(ImAxis_Y1).Max - (up.Max + downwards)) <= 1e-9 * up.Size());
}
