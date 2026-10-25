#include "plot_view.h"

#include <imgui.h>
#include <implot.h>
#include <implot_internal.h>

namespace praxis::scene {

namespace {

bool over(const ImPlotPlot &drawn, ImVec2 at)
{
    return drawn.PlotRect.Contains(at) || drawn.Axes[ImAxis_X1].HoverRect.Contains(at) || drawn.Axes[ImAxis_Y1].HoverRect.Contains(at);
}

bool pointed_at(const ImPlotPlot &drawn)
{
    return ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) && over(drawn, ImGui::GetIO().MousePos);
}

// A press dragged no further than the GUI library's drag threshold stays a click.
bool moved_by_pointer(const ImPlotPlot &drawn)
{
    const bool held = drawn.Held || drawn.Axes[ImAxis_X1].Held || drawn.Axes[ImAxis_Y1].Held;
    if(held && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
        return true;

    return pointed_at(drawn) && (ImGui::GetIO().MouseWheel != 0.f || ImGui::IsMouseClicked(ImGuiMouseButton_Right));
}

}

bool returns_to_fitted(const plot_limits &moved_from, const plot_limits &held, bool double_clicked)
{
    return double_clicked || moved_from.lower != held.lower || moved_from.upper != held.upper;
}

bool shows_fitted(plot_view &view, const plot_frame &shown, const char *plot_id)
{
    if(!shown.abscissa_limits)
        return false;

    const ImPlotPlot *const drawn = ImPlot::GetPlot(plot_id);
    const bool double_clicked     = drawn != nullptr && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && pointed_at(*drawn);
    if(view && returns_to_fitted(*view, *shown.abscissa_limits, double_clicked))
        view.reset();
    else if(!view && drawn != nullptr && moved_by_pointer(*drawn))
        view = shown.abscissa_limits;

    return !view;
}

}
