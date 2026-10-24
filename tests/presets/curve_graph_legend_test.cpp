#include "imgui_frame.h"
#include "composed_optimization.h"

#include "praxis/optimization/registered.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/store.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <implot.h>
#include <imgui_internal.h>
#include <implot_internal.h>

#include <memory>
#include <string>
#include <vector>
#include <utility>

#ifndef PRAXIS_SHIPPED_OPTIMIZATION_DIR
    #define PRAXIS_SHIPPED_OPTIMIZATION_DIR ""
#endif

using namespace praxis;
using namespace praxis::fixture;

namespace {

const char *const graph_name = "Graph##curve";

double cubic(double x)
{
    return x * x * x - 2.0 * x - 5.0;
}

double cubic_slope(double x)
{
    return 3.0 * x * x - 2.0;
}

double quartic(double x)
{
    return x * x * x * x - 3.0 * x * x + x;
}

double quartic_slope(double x)
{
    return 4.0 * x * x * x - 6.0 * x + 1.0;
}

double quartic_curvature(double x)
{
    return 12.0 * x * x - 6.0;
}

std::vector<optimization::registered_curve> shipped_curves()
{
    return {optimization::registered_curve("x^3 - 2x - 5", &cubic, &cubic_slope), optimization::registered_curve("x^4 - 3x^2 + x", &quartic, &quartic_slope, &quartic_curvature)};
}

struct drawn_graph
{
    ImRect frame;
    ImRect visible;
    ImRect inner;
    ImVec2 reach;
    bool native_legend;
    float line;
    std::string written;
};

std::shared_ptr<scene::imgui_window> graph_of(const composed_optimization &composed)
{
    for(const std::shared_ptr<scene::imgui_window> &window : composed.preset->windows)
        if(window->display_name() == graph_name)
            return window;
    FAIL("no window is named " << graph_name);

    return nullptr;
}

drawn_graph read_back(std::string written)
{
    const ImPlotPlot &plot   = *ImPlot::GetCurrentContext()->Plots.GetByIndex(0);
    const ImGuiWindow &panel = *ImGui::FindWindowByName(graph_name);
    const ImRect visible(ImMax(plot.PlotRect.Min, panel.InnerClipRect.Min), ImMin(plot.PlotRect.Max, panel.InnerClipRect.Max));

    return drawn_graph{plot.FrameRect, visible, panel.InnerClipRect, panel.DC.CursorMaxPos, (plot.Flags & ImPlotFlags_NoLegend) == 0, ImGui::GetTextLineHeight(), std::move(written)};
}

// The Graph at the rectangle its opening placement gives it on a 1920 x 1080 work area at font
// scale 1, drawn once more into the log so what it writes as text is read.
drawn_graph drawn_at_opening(const std::string &document, const std::string &preset)
{
    const composed_optimization composed(config::resolve(document, PRAXIS_SHIPPED_OPTIMIZATION_DIR), shipped_curves(), preset);
    const std::shared_ptr<scene::imgui_window> graph = graph_of(composed);
    REQUIRE(graph->opening().has_value());

    tests::imgui_frame frames;
    const scene::window_rectangle at = scene::opening_rectangle(*graph->opening(), ImVec2(0.f, 0.f), ImVec2(1920.f, 1080.f), 13.f);
    frames.draw(
            [&graph, &at]
            {
                ImGui::SetNextWindowPos(at.position, ImGuiCond_Always);
                ImGui::SetNextWindowSize(at.size, ImGuiCond_Always);
                graph->render();
            },
            4);

    return read_back(composed.written(frames, graph_name));
}

void check_labels_in_full(const drawn_graph &drawn, const std::vector<std::string> &labels)
{
    for(const std::string &label : labels)
    {
        INFO(label);
        CHECK(drawn.written.find(label) != std::string::npos);
    }
    CHECK(drawn.reach.x <= drawn.inner.Max.x);
    CHECK(drawn.reach.y <= drawn.inner.Max.y);
}

void check_clear_of_the_key(const drawn_graph &drawn, float width, float height)
{
    CHECK_FALSE(drawn.native_legend);
    CHECK(drawn.reach.y >= drawn.frame.Max.y + drawn.line);
    CHECK(drawn.visible.GetWidth() >= width - 2.f);
    CHECK(drawn.visible.GetHeight() >= height - 2.f);
}

}

TEST_CASE("the root cubic's Graph at its opening rectangle writes its legend in full and keeps its plot clear", "[presets][optimization]")
{
    const drawn_graph drawn = drawn_at_opening("curve.xml", "Root finding: a cubic");

    check_labels_in_full(drawn, {"g(x)", "Iterates", "x_k", "root", "tangent at x_k", "zero of the tangent", "x_(k+1)", "linearization point p", "tangent at p"});
    check_clear_of_the_key(drawn, 218.f, 140.f);
}

TEST_CASE("the minimized quartic's Graph at its opening rectangle writes its legend in full and keeps its plot clear", "[presets][optimization]")
{
    const drawn_graph drawn = drawn_at_opening("curve-minimum.xml", "Minimization: x^4 - 3x^2 + x");

    check_labels_in_full(drawn, {"f(x)", "Iterates", "x_k", "stationary point", "vertex", "quadratic model at x_k", "x_(k+1)", "linearization point p", "tangent at p"});
    check_clear_of_the_key(drawn, 225.f, 123.f);
}
