#include "imgui_frame.h"

#include "praxis/scene/plot_window.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <implot.h>
#include <imgui_internal.h>
#include <implot_internal.h>

#include <string>
#include <vector>
#include <optional>

using namespace praxis;

namespace {

struct drawn_legend
{
    ImRect plot;
    ImRect legend;
    ImVec2 padding;
};

scene::plot_reading two_named_series()
{
    const scene::plot_series rising{"rising", {0.0, 1.0, 2.0, 3.0}, {0.0, 1.0, 2.0, 3.0}, scene::plot_mark::line};
    const scene::plot_series falling{"falling", {0.0, 1.0, 2.0, 3.0}, {3.0, 2.0, 1.0, 0.0}, scene::plot_mark::line};

    return scene::plot_reading{"", "s", {scene::plot_frame{"value", false, {rising, falling}, std::nullopt}}};
}

drawn_legend drawn_unplaced()
{
    scene::plot_window panel("Plot", nullptr, [] { return two_named_series(); });

    tests::imgui_frame frames;
    frames.draw(
            [&panel]
            {
                ImGui::SetNextWindowPos(ImVec2(0.f, 0.f));
                ImGui::SetNextWindowSize(ImVec2(600.f, 400.f));
                panel.render();
            },
            3);

    const ImPlotPlot &plot = *ImPlot::GetCurrentContext()->Plots.GetByIndex(0);

    return drawn_legend{plot.PlotRect, plot.Items.Legend.RectClamped, ImPlot::GetStyle().LegendPadding};
}

struct drawn_key
{
    ImRect frame;
    ImRect inner;
    ImVec2 reach;
    float content_right;
    float line;
    bool native_legend;
    std::string written;
};

// A 300 x 400 px panel drawn until it settles, then once more into the log.
drawn_key key_drawn(tests::imgui_frame &frames, scene::plot_window &panel)
{
    const auto draw = [&panel]
    {
        ImGui::SetNextWindowPos(ImVec2(0.f, 0.f));
        ImGui::SetNextWindowSize(ImVec2(300.f, 400.f));
        panel.render();
    };
    frames.draw(draw, 3);

    std::string written;
    frames.draw_frame(
            [&draw, &written]
            {
                ImGui::LogToBuffer();
                draw();
                written = GImGui->LogBuffer.c_str();
                ImGui::LogFinish();
            });
    const ImPlotPlot &plot   = *ImPlot::GetCurrentContext()->Plots.GetByIndex(0);
    const ImGuiWindow &shown = *ImGui::FindWindowByName("Plot");

    return drawn_key{plot.FrameRect, shown.InnerClipRect, shown.DC.CursorMaxPos, shown.WorkRect.Max.x, ImGui::GetTextLineHeight(), (plot.Flags & ImPlotFlags_NoLegend) == 0, written};
}

}

TEST_CASE("a plot window given no legend placement keeps its legend inside the plot at the upper left", "[scene]")
{
    const drawn_legend drawn = drawn_unplaced();

    CHECK(drawn.legend.Min.x == drawn.plot.Min.x + drawn.padding.x);
    CHECK(drawn.legend.Min.y == drawn.plot.Min.y + drawn.padding.y);
    CHECK(drawn.plot.Contains(drawn.legend));
}

TEST_CASE("a plot window given a wrapped key writes every entry under the plot, wrapped to its content width", "[scene]")
{
    const std::vector<std::string> names{"first series", "second series", "third series", "fourth series", "fifth series", "sixth series"};
    scene::plot_frame shown{"value", false, {}, std::nullopt};
    for(const std::string &name : names)
        shown.series.push_back(scene::plot_series{name, {0.0, 1.0}, {0.0, static_cast<double>(shown.series.size())}, scene::plot_mark::line});
    scene::plot_window panel("Plot", nullptr, [shown] { return scene::plot_reading{"", "s", {shown}}; });
    panel.legend_at(scene::plot_legend::wrapped_key);

    tests::imgui_frame frames;
    const drawn_key drawn = key_drawn(frames, panel);

    CHECK_FALSE(drawn.native_legend);
    for(const std::string &name : names)
        CHECK(drawn.written.find(name) != std::string::npos);
    CHECK(drawn.reach.y >= drawn.frame.Max.y + 2.f * drawn.line);
    CHECK(drawn.reach.y <= drawn.inner.Max.y);
    CHECK(drawn.reach.x <= drawn.content_right);
}
