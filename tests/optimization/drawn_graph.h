#ifndef HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_DRAWN_GRAPH_H
#define HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_DRAWN_GRAPH_H

#include "composed_curve.h"

#include "imgui_frame.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/plot_window.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <implot.h>
#include <implot_internal.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <functional>

namespace praxis::fixture {

inline scene::plot_frame drawn_now(const opened &taken)
{
    const scene::plot_reading shown = curve_reading(taken.inputs->seen);
    REQUIRE(shown.frames.size() == 1u);

    return shown.frames.front();
}

inline const scene::plot_series *named(const scene::plot_frame &frame, const std::string &name)
{
    const auto found = std::find_if(frame.series.begin(), frame.series.end(), [&name](const scene::plot_series &series) { return series.name == name; });

    return found == frame.series.end() ? nullptr : &*found;
}

// Both ends of a two-point line lie on the line through (x, g) with the slope given.
inline void check_on_line(const scene::plot_series &line, double x, double g, double slope)
{
    CHECK(line.mark == scene::plot_mark::line);
    REQUIRE(line.abscissa.size() == 2u);
    REQUIRE(line.ordinate.size() == 2u);
    CHECK(line.abscissa[0] < line.abscissa[1]);
    for(std::size_t end = 0; end < 2u; ++end)
        CHECK(std::abs(line.ordinate[end] - (g + slope * (line.abscissa[end] - x))) <= 1.0e-9 * std::max(1.0, std::abs(line.ordinate[end])));
}

// The finite ordinates of the frame's first `leading` series at abscissae within its held limits.
inline interval drawn_span(const scene::plot_frame &frame, std::size_t leading)
{
    REQUIRE(frame.abscissa_limits.has_value());
    interval span{std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
    for(std::size_t at = 0; at < leading && at < frame.series.size(); ++at)
        for(std::size_t point = 0; point < frame.series[at].ordinate.size(); ++point)
        {
            const double x = frame.series[at].abscissa[point];
            const double y = frame.series[at].ordinate[point];
            if(x >= frame.abscissa_limits->lower && x <= frame.abscissa_limits->upper && std::isfinite(y))
                span = interval{std::min(span.lower, y), std::max(span.upper, y)};
        }

    return span;
}

// Composes over one curve, the graph alone unless a composer is handed in, and tears the preset
// down when the case ends.
struct graph_stage
{
    scheduler::scheduler loop;
    staged built;
    opened taken;
    std::shared_ptr<const scenario_snapshot> ran;

    graph_stage(const registered_curve &curve, const capabilities &solving, const scenario_settings &settings)
            : loop(scheduler::inline_workers)
            , built(loop)
            , taken(open_with(built.site, {curve}, solving, settings))
            , ran(settled(loop, taken))
    {
    }

    graph_stage(const registered_curve &curve, const scenario_window_composer &windows, bool checks_derivative)
            : loop(scheduler::inline_workers)
            , built(loop)
            , taken(open_composing(built.site, {curve}, baseline(), opening(curve.name), windows, checks_derivative))
            , ran(settled(loop, taken))
    {
    }

    ~graph_stage()
    {
        taken.composed->tear_down();
    }

    scene::plot_frame at_step(std::uint32_t step)
    {
        command(taken.inputs->scenario, [step](scenario_state &state) { state.select_step(step); });

        return drained();
    }

    scene::plot_frame drained()
    {
        REQUIRE(loop.drain().has_value());

        return drawn_now(taken);
    }
};

struct pressed_at
{
    double abscissa;
    double pixel;
};

// Three quarters of the way across and down a plot large enough to keep the legend in its upper
// left corner clear of that spot. The modifier and the pointer are set a frame before the press,
// which the library sees only then.
inline pressed_at shift_press(const std::shared_ptr<scene::imgui_window> &graph)
{
    tests::imgui_frame frames;
    const std::function<void()> draw = [&graph]
    {
        ImGui::SetNextWindowSize(ImVec2(800.f, 600.f));
        graph->render();
    };
    frames.draw(draw);

    const ImPlotPlot &plot    = *ImPlot::GetCurrentContext()->Plots.GetByIndex(0);
    const ImPlotRange &across = plot.Axes[ImAxis_X1].Range;
    const pressed_at expected{across.Min + 0.75 * across.Size(), across.Size() / plot.PlotRect.GetWidth()};

    ImGui::GetIO().AddMousePosEvent(plot.PlotRect.Min.x + 0.75f * plot.PlotRect.GetWidth(), plot.PlotRect.Min.y + 0.75f * plot.PlotRect.GetHeight());
    ImGui::GetIO().AddKeyEvent(ImGuiMod_Shift, true);
    frames.draw_frame(draw);
    ImGui::GetIO().AddMouseButtonEvent(ImGuiMouseButton_Left, true);
    frames.draw_frame(draw);
    ImGui::GetIO().AddMouseButtonEvent(ImGuiMouseButton_Left, false);
    frames.draw_frame(draw);

    return expected;
}

}

#endif
