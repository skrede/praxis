#include "plot_ring.h"
#include "imgui_frame.h"

#include "praxis/scene/plot_window.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <implot.h>
#include <imgui_internal.h>
#include <implot_internal.h>

#include <vector>
#include <optional>
#include <algorithm>
#include <functional>

using namespace praxis;

namespace {

const scene::plot_series red{"red", {0.0, 1.0}, {0.0, 1.0}, scene::plot_mark::line, 0xff0000};
const scene::plot_series blue{"blue", {0.0, 1.0}, {1.0, 0.0}, scene::plot_mark::markers, 0x0000ff};

void drawn_at(tests::imgui_frame &frames, scene::plot_window &panel, ImVec2 size, float scroll)
{
    panel.legend_at(scene::plot_legend::wrapped_key);
    frames.draw(
            [&panel, size, scroll]
            {
                ImGui::SetNextWindowPos(ImVec2(0.f, 0.f));
                ImGui::SetNextWindowSize(size);
                ImGui::SetNextWindowScroll(ImVec2(0.f, scroll));
                panel.render();
            },
            3);
}

// The colors the panel's geometry under `below` is drawn in, which are its key's swatches and text.
std::vector<ImU32> colors_under(float below)
{
    std::vector<ImU32> colors;
    for(const ImDrawVert &corner : ImGui::FindWindowByName("Plot")->DrawList->VtxBuffer)
        if(corner.pos.y > below)
            colors.push_back(corner.col);

    return colors;
}

bool holds(const std::vector<ImU32> &colors, ImU32 color)
{
    return std::ranges::find(colors, color) != colors.end();
}

struct keyed_series
{
    std::vector<ImU32> keyed;
    std::vector<ImU32> plotted;
    std::vector<ImU32> colormap;
};

// One frame of the series in a 300 x 400 px panel keyed under its plot.
keyed_series keyed_in(const std::vector<scene::plot_series> &series)
{
    scene::plot_window panel("Plot", nullptr, [series] { return scene::plot_reading{"", "s", {scene::plot_frame{"value", false, series, std::nullopt}}}; });
    tests::imgui_frame frames;
    drawn_at(frames, panel, ImVec2(300.f, 400.f), 0.f);

    ImPlotPlot &plot = *ImPlot::GetCurrentContext()->Plots.GetByIndex(0);
    keyed_series drawn{colors_under(plot.FrameRect.Max.y), {}, {}};
    for(int at = 0; at < plot.Items.GetItemCount(); ++at)
    {
        drawn.plotted.push_back(plot.Items.GetItemByIndex(at)->Color);
        drawn.colormap.push_back(ImGui::GetColorU32(ImPlot::GetColormapColor(at)));
    }

    return drawn;
}

}

TEST_CASE("a plot window keys and plots a series given a tone in that tone", "[scene]")
{
    const keyed_series drawn = keyed_in({red, blue});

    CHECK(holds(drawn.keyed, IM_COL32(255, 0, 0, 255)));
    CHECK(holds(drawn.keyed, IM_COL32(0, 0, 255, 255)));
    CHECK(drawn.plotted == std::vector<ImU32>{IM_COL32(255, 0, 0, 255), IM_COL32(0, 0, 255, 255)});
}

TEST_CASE("a plot window keys and plots series given no tone in the colormap's order", "[scene]")
{
    const keyed_series drawn =
            keyed_in({scene::plot_series{"rising", {0.0, 1.0}, {0.0, 1.0}, scene::plot_mark::line}, scene::plot_series{"falling", {0.0, 1.0}, {1.0, 0.0}, scene::plot_mark::line}});

    REQUIRE(drawn.colormap.size() == 2u);
    CHECK(drawn.plotted == drawn.colormap);
    CHECK(holds(drawn.keyed, drawn.colormap[0]));
    CHECK(holds(drawn.keyed, drawn.colormap[1]));
}

TEST_CASE("a ring series given a tone registers one item in that tone", "[scene]")
{
    const keyed_series drawn = keyed_in({scene::plot_series{"ring", {1.0}, {1.0}, scene::plot_mark::ring, 0x00ff00}});

    CHECK(drawn.plotted == std::vector<ImU32>{IM_COL32(0, 255, 0, 255)});
    CHECK(scene::ring_radius_ratio == 1.75f);
    CHECK(scene::ring_weight == 1.f);
}

TEST_CASE("a plot clipped out of view keys its toned series in their tones", "[scene]")
{
    scene::plot_reading shown{"", "s", {scene::plot_frame{"first", false, {red, blue}, std::nullopt}}};
    for(const char *frame : {"second", "third"})
        shown.frames.push_back(scene::plot_frame{frame, false, {scene::plot_series{frame, {0.0, 1.0}, {0.0, 1.0}, scene::plot_mark::line}}, std::nullopt});
    scene::plot_window panel("Plot", nullptr, [shown] { return shown; });

    tests::imgui_frame frames;
    drawn_at(frames, panel, ImVec2(300.f, 200.f), 10000.f);
    const ImRect first = ImPlot::GetCurrentContext()->Plots.GetByIndex(0)->FrameRect;
    REQUIRE(first.Max.y < ImGui::FindWindowByName("Plot")->InnerClipRect.Min.y);

    const std::vector<ImU32> keyed = colors_under(first.Max.y);
    CHECK(holds(keyed, IM_COL32(255, 0, 0, 255)));
    CHECK(holds(keyed, IM_COL32(0, 0, 255, 255)));
}
