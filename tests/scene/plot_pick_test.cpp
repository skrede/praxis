#include "imgui_frame.h"

#include "praxis/scene/plot_window.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <implot.h>
#include <implot_internal.h>

#include <cmath>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <functional>

using namespace praxis;

namespace {

using drawing = std::function<void()>;

struct picked
{
    std::size_t frame;
    double abscissa;
    double ordinate;
};

const char *const title = "Plot";

scene::plot_reading stacked_frames(std::size_t count)
{
    const scene::plot_series rising{"rising", {0.0, 1.0, 2.0, 3.0}, {1.0, 3.0, 5.0, 9.0}, scene::plot_mark::line};

    scene::plot_reading shown{"", "s", {}};
    for(std::size_t frame = 0; frame < count; ++frame)
        shown.frames.push_back(scene::plot_frame{"value", false, {rising}, std::nullopt});

    return shown;
}

scene::plot_source answering(scene::plot_reading given)
{
    return [given = std::move(given)] { return given; };
}

scene::plot_pick recording(std::vector<picked> &taken)
{
    return [&taken](std::size_t frame, double abscissa, double ordinate) { taken.push_back(picked{frame, abscissa, ordinate}); };
}

// The plots the fixture's one context holds, in the order they were first drawn.
const ImPlotPlot &drawn_plot(int index)
{
    return *ImPlot::GetCurrentContext()->Plots.GetByIndex(index);
}

// The pointer is placed and the modifier set a frame before the press, and the press is released a
// frame after it, because the library sees neither a press nor a release within the frame it moves in.
void press_at(tests::imgui_frame &frames, const drawing &draw, ImVec2 spot, bool shifted)
{
    ImGuiIO &io = ImGui::GetIO();
    io.AddMousePosEvent(spot.x, spot.y);
    io.AddKeyEvent(ImGuiMod_Shift, shifted);
    frames.draw_frame(draw);
    io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
    frames.draw_frame(draw);
    io.AddMouseButtonEvent(ImGuiMouseButton_Left, false);
    frames.draw_frame(draw);
}

// The plot coordinates of a pixel, read off the plot's rectangle and its two axis ranges; the
// ordinate grows upwards while the pixel rows grow downwards.
picked coordinates_at(const ImPlotPlot &plot, ImVec2 spot)
{
    const ImPlotRange &across = plot.Axes[ImAxis_X1].Range;
    const ImPlotRange &up     = plot.Axes[ImAxis_Y1].Range;
    const double right        = (spot.x - plot.PlotRect.Min.x) / plot.PlotRect.GetWidth();
    const double down         = (spot.y - plot.PlotRect.Min.y) / plot.PlotRect.GetHeight();

    return picked{0u, across.Min + right * across.Size(), up.Max - down * up.Size()};
}

}

TEST_CASE("a left press with Shift held over a plot hands the pick the plot coordinates under it", "[scene]")
{
    std::vector<picked> taken;
    scene::plot_window panel(title, nullptr, answering(stacked_frames(1u)), recording(taken));
    const drawing draw = [&panel] { panel.render(); };

    tests::imgui_frame frames;
    frames.draw(draw);
    const ImVec2 center = drawn_plot(0).PlotRect.GetCenter();
    press_at(frames, draw, center, true);

    REQUIRE(taken.size() == 1u);
    const ImPlotPlot &plot = drawn_plot(0);
    const picked expected  = coordinates_at(plot, center);
    CHECK(taken.front().frame == 0u);
    CHECK(std::abs(taken.front().abscissa - expected.abscissa) <= plot.Axes[ImAxis_X1].Range.Size() / plot.PlotRect.GetWidth());
    CHECK(std::abs(taken.front().ordinate - expected.ordinate) <= plot.Axes[ImAxis_Y1].Range.Size() / plot.PlotRect.GetHeight());
}

TEST_CASE("a left press without Shift over a plot hands the pick nothing", "[scene]")
{
    std::vector<picked> taken;
    scene::plot_window panel(title, nullptr, answering(stacked_frames(1u)), recording(taken));
    const drawing draw = [&panel] { panel.render(); };

    tests::imgui_frame frames;
    frames.draw(draw);
    press_at(frames, draw, drawn_plot(0).PlotRect.GetCenter(), false);

    CHECK(taken.empty());
}

TEST_CASE("a left press with Shift held beside the plot rather than over it hands the pick nothing", "[scene]")
{
    std::vector<picked> taken;
    scene::plot_window panel(title, nullptr, answering(stacked_frames(1u)), recording(taken));
    const drawing draw = [&panel] { panel.render(); };

    tests::imgui_frame frames;
    frames.draw(draw);
    const ImPlotPlot &plot = drawn_plot(0);
    press_at(frames, draw, ImVec2(plot.FrameRect.Min.x + 1.f, plot.PlotRect.GetCenter().y), true);
    press_at(frames, draw, ImVec2(1900.f, 1060.f), true);

    CHECK(taken.empty());
}

TEST_CASE("a plot window given a pick that is never pressed draws as one given none", "[scene]")
{
    std::vector<picked> taken;
    scene::plot_window picking(title, nullptr, answering(stacked_frames(2u)), recording(taken));
    scene::plot_window plain(title, nullptr, answering(stacked_frames(2u)));

    std::size_t picking_drawn = 0u;
    {
        tests::imgui_frame with_pick;
        with_pick.draw([&picking] { picking.render(); });
        picking_drawn = with_pick.signature();
    }

    tests::imgui_frame without_pick;
    without_pick.draw([&plain] { plain.render(); });

    CHECK(picking_drawn == without_pick.signature());
    CHECK(taken.empty());
}

TEST_CASE("a Shift press over the second of two stacked plots names the second frame", "[scene]")
{
    std::vector<picked> taken;
    scene::plot_window panel(title, nullptr, answering(stacked_frames(2u)), recording(taken));
    const drawing draw = [&panel] { panel.render(); };

    tests::imgui_frame frames;
    frames.draw(draw);
    press_at(frames, draw, drawn_plot(1).PlotRect.GetCenter(), true);

    REQUIRE(taken.size() == 1u);
    CHECK(taken.front().frame == 1u);
}
