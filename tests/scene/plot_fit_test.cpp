#include "imgui_frame.h"

#include "praxis/scene/plot_window.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <implot.h>
#include <imgui_internal.h>
#include <implot_internal.h>

#include <string>
#include <functional>
#include <initializer_list>

using namespace praxis;

namespace {

const char *const title = "Plot";

// A frame held at [0, 1] with a fitted line through (0, 0) and (1, 1), and a steep second series.
scene::plot_reading steep_beside(scene::plot_mark marked, bool fitted)
{
    scene::plot_series steep{"steep", {0.0, 1.0}, {-100.0, 100.0}, marked};
    steep.fitted = fitted;

    return scene::plot_reading{
            "", "s", {scene::plot_frame{"value", false, {scene::plot_series{"rising", {0.0, 1.0}, {0.0, 1.0}, scene::plot_mark::line}, steep}, scene::plot_limits{0.0, 1.0}}}};
}

ImPlotRange fitted_ordinate(scene::plot_mark marked, bool fitted)
{
    scene::plot_window panel(title, nullptr, [marked, fitted] { return steep_beside(marked, fitted); });
    const std::function<void()> draw = [&panel]
    {
        ImGui::SetNextWindowPos(ImVec2(0.f, 0.f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(900.f, 600.f), ImGuiCond_Always);
        panel.render();
    };

    tests::imgui_frame frames;
    frames.draw(draw, 3);
    ImGuiWindow *const drawn = ImGui::FindWindowByName(title);
    REQUIRE(drawn != nullptr);
    const ImPlotPlot *plot = ImPlot::GetCurrentContext()->Plots.GetByKey(drawn->GetID("##plot0"));
    REQUIRE(plot != nullptr);
    CHECK(plot->Axes[ImAxis_X1].Range.Min == 0.0);
    CHECK(plot->Axes[ImAxis_X1].Range.Max == 1.0);

    return plot->Axes[ImAxis_Y1].Range;
}

}

TEST_CASE("a series not fitted is left out of the fitted ordinate, and the same series fitted widens it", "[scene]")
{
    for(const scene::plot_mark marked : {scene::plot_mark::line, scene::plot_mark::segments})
    {
        const ImPlotRange unfitted = fitted_ordinate(marked, false);
        CHECK(unfitted.Min >= -1.0);
        CHECK(unfitted.Max <= 2.0);

        const ImPlotRange widened = fitted_ordinate(marked, true);
        CHECK(widened.Min <= -100.0);
        CHECK(widened.Max >= 100.0);
    }
}
