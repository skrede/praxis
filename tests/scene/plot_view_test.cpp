#include "imgui_frame.h"

#include "praxis/scene/plot_window.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <implot.h>
#include <imgui_internal.h>
#include <implot_internal.h>

#include <cmath>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <functional>

using namespace praxis;

namespace {

using drawing = std::function<void()>;

const char *const title = "Plot";

scene::plot_reading rising(std::size_t frames, std::optional<scene::plot_limits> held, double scale = 1.0)
{
    const scene::plot_series curve{
            "rising", {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}, {scale, 3.0 * scale, 5.0 * scale, 9.0 * scale, 11.0 * scale, 15.0 * scale, 20.0 * scale}, scene::plot_mark::line};

    scene::plot_reading shown{"", "s", {}};
    for(std::size_t frame = 0; frame < frames; ++frame)
        shown.frames.push_back(scene::plot_frame{"value", false, {curve}, held});

    return shown;
}

// A plot as the library keeps it between frames, found under the identifier the window draws it by.
const ImPlotPlot &plot_in(const char *window, std::size_t frame)
{
    ImGuiWindow *const drawn = ImGui::FindWindowByName(window);
    REQUIRE(drawn != nullptr);
    const ImPlotPlot *plot = ImPlot::GetCurrentContext()->Plots.GetByKey(drawn->GetID(("##plot" + std::to_string(frame)).c_str()));
    REQUIRE(plot != nullptr);

    return *plot;
}

ImPlotRange range_of(std::size_t frame, ImAxis axis)
{
    return plot_in(title, frame).Axes[axis].Range;
}

drawing placed(scene::plot_window &panel)
{
    return [&panel]
    {
        ImGui::SetNextWindowPos(ImVec2(0.f, 0.f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(900.f, 600.f), ImGuiCond_Always);
        panel.render();
    };
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

void wheel_over(tests::imgui_frame &frames, const drawing &draw, std::size_t frame)
{
    point_at(frames, draw, plot_in(title, frame).PlotRect.GetCenter());
    ImGui::GetIO().AddMouseWheelEvent(0.f, 1.f);
    frames.draw(draw, 3);
}

// Each press and each move takes a frame of its own, because the library sees neither a press nor a
// release within the frame the pointer moves in.
void press_at(tests::imgui_frame &frames, const drawing &draw, ImGuiMouseButton button, ImVec2 spot, ImVec2 drag, bool shifted = false)
{
    ImGuiIO &io = ImGui::GetIO();
    io.AddMousePosEvent(spot.x, spot.y);
    io.AddKeyEvent(ImGuiMod_Shift, shifted);
    frames.draw_frame(draw);
    io.AddMouseButtonEvent(button, true);
    frames.draw_frame(draw);
    point_at(frames, draw, ImVec2(spot.x + drag.x, spot.y + drag.y));
    io.AddMouseButtonEvent(button, false);
    frames.draw_frame(draw);
    io.AddKeyEvent(ImGuiMod_Shift, false);
    frames.draw(draw, 3);
}

bool narrowed(const ImPlotRange &moved, const ImPlotRange &fitted)
{
    return moved.Size() < fitted.Size() && moved.Min >= fitted.Min && moved.Max <= fitted.Max;
}

}

TEST_CASE("a box selection over a frame with held limits narrows both of its ranges, and the view stays", "[scene]")
{
    scene::plot_window panel(title, nullptr, [] { return rising(1u, scene::plot_limits{0.0, 3.0}); });
    const drawing draw = placed(panel);

    tests::imgui_frame frames;
    frames.draw(draw);
    const ImPlotRange across = range_of(0u, ImAxis_X1);
    const ImPlotRange up     = range_of(0u, ImAxis_Y1);
    const ImVec2 middle      = plot_in(title, 0u).PlotRect.GetCenter();
    press_at(frames, draw, ImGuiMouseButton_Right, ImVec2(middle.x - 40.f, middle.y - 30.f), ImVec2(80.f, 60.f));

    const ImPlotRange boxed = range_of(0u, ImAxis_X1);
    CHECK(narrowed(boxed, across));
    CHECK(narrowed(range_of(0u, ImAxis_Y1), up));
    frames.draw(draw, 4);
    CHECK(same(range_of(0u, ImAxis_X1), boxed));
}

TEST_CASE("a moved view returns to the fitted view of held limits that change", "[scene]")
{
    std::optional<scene::plot_limits> held = scene::plot_limits{0.0, 3.0};
    scene::plot_window panel(title, nullptr, [&held] { return rising(1u, held); });
    const drawing draw = placed(panel);

    tests::imgui_frame frames;
    frames.draw(draw);
    wheel_over(frames, draw, 0u);
    REQUIRE(narrowed(range_of(0u, ImAxis_X1), ImPlotRange(0.0, 3.0)));

    held = scene::plot_limits{0.0, 6.0};
    frames.draw(draw, 3);
    CHECK(same(range_of(0u, ImAxis_X1), ImPlotRange(0.0, 6.0)));
    CHECK(same(range_of(0u, ImAxis_Y1), ImPlotRange(1.0, 20.0)));
}

TEST_CASE("a Shift-click in a moved view picks the coordinates under the pointer in that view and moves nothing", "[scene]")
{
    std::vector<ImPlotPoint> taken;
    scene::plot_window panel(
            title, nullptr, [] { return rising(1u, scene::plot_limits{0.0, 3.0}); },
            [&taken](std::size_t, double abscissa, double ordinate) { taken.emplace_back(abscissa, ordinate); });
    const drawing draw = placed(panel);

    tests::imgui_frame frames;
    frames.draw(draw);
    wheel_over(frames, draw, 0u);
    const ImPlotRange across = range_of(0u, ImAxis_X1);
    const ImPlotRange up     = range_of(0u, ImAxis_Y1);
    REQUIRE(narrowed(across, ImPlotRange(0.0, 3.0)));

    const ImRect area(plot_in(title, 0u).PlotRect);
    const ImVec2 spot(area.Min.x + area.GetWidth() * 0.25f, area.Min.y + area.GetHeight() * 0.75f);
    press_at(frames, draw, ImGuiMouseButton_Left, spot, ImVec2(0.f, 0.f), true);

    REQUIRE(taken.size() == 1u);
    CHECK(std::abs(taken.front().x - (across.Min + 0.25 * across.Size())) <= across.Size() / area.GetWidth());
    CHECK(std::abs(taken.front().y - (up.Max - 0.75 * up.Size())) <= up.Size() / area.GetHeight());
    CHECK(same(range_of(0u, ImAxis_X1), across));
    CHECK(same(range_of(0u, ImAxis_Y1), up));
}

TEST_CASE("moving the first of two stacked frames with held limits leaves the second following its fitted view", "[scene]")
{
    double scale = 1.0;
    scene::plot_window panel(title, nullptr, [&scale] { return rising(2u, scene::plot_limits{0.0, 3.0}, scale); });
    const drawing draw = placed(panel);

    tests::imgui_frame frames;
    frames.draw(draw);
    const ImPlotRange second = range_of(1u, ImAxis_Y1);
    wheel_over(frames, draw, 0u);
    const ImPlotRange moved = range_of(0u, ImAxis_X1);
    REQUIRE(narrowed(moved, ImPlotRange(0.0, 3.0)));

    scale = 2.0;
    frames.draw(draw, 3);
    CHECK(same(range_of(0u, ImAxis_X1), moved));
    CHECK(same(range_of(1u, ImAxis_X1), ImPlotRange(0.0, 3.0)));
    CHECK(range_of(1u, ImAxis_Y1).Max == 2.0 * second.Max);
}

TEST_CASE("a frame without held limits fits its data, keeps a zoom, and fits its data again on a double click", "[scene]")
{
    scene::plot_window panel(title, nullptr, [] { return rising(1u, std::nullopt); });
    const drawing draw = placed(panel);

    tests::imgui_frame frames;
    frames.draw(draw);
    const ImPlotRange fitted = range_of(0u, ImAxis_X1);
    CHECK(fitted.Min <= 0.0);
    CHECK(fitted.Max >= 6.0);

    wheel_over(frames, draw, 0u);
    const ImPlotRange zoomed = range_of(0u, ImAxis_X1);
    CHECK(narrowed(zoomed, fitted));
    frames.draw(draw, 4);
    CHECK(same(range_of(0u, ImAxis_X1), zoomed));

    for(int click = 0; click < 2; ++click)
        press_at(frames, draw, ImGuiMouseButton_Left, plot_in(title, 0u).PlotRect.GetCenter(), ImVec2(0.f, 0.f));
    CHECK(same(range_of(0u, ImAxis_X1), fitted));
}
