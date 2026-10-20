#include "praxis/scene/plot_window.h"

#include <imgui.h>
#include <implot.h>

#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>
#include <functional>

namespace praxis::scene {

namespace {

const char *const absent_context_line = "No plotting context is current, so no curve is drawn.";

// The plotting library counts points in an int.
int drawn_length(const plot_series &curve)
{
    return static_cast<int>(std::min(curve.abscissa.size(), curve.ordinate.size()));
}

// The frames of one reading share what the panel leaves, less the spacing the GUI library puts
// between successive items. The plotting library raises a plot given the fill sentinel to its own
// minimum but not one given a height, so that floor is applied here instead.
float shared_height(std::size_t count)
{
    const float between = ImGui::GetStyle().ItemSpacing.y * static_cast<float>(count - 1u);
    const float share   = (ImGui::GetContentRegionAvail().y - between) / static_cast<float>(count);

    return std::max(share, ImPlot::GetStyle().PlotMinSize.y);
}

void setup_axes(const plot_frame &shown, const std::string &abscissa_label)
{
    const ImPlotAxisFlags fitted = shown.abscissa_limits ? ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit : ImPlotAxisFlags_None;
    ImPlot::SetupAxes(abscissa_label.c_str(), shown.ordinate_label.c_str(), ImPlotAxisFlags_None, fitted);
    if(shown.abscissa_limits)
        ImPlot::SetupAxisLimits(ImAxis_X1, shown.abscissa_limits->lower, shown.abscissa_limits->upper, ImPlotCond_Always);
    if(shown.logarithmic_ordinate)
        ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
    ImPlot::SetupLegend(ImPlotLocation_NorthWest);
}

void render_curve(const plot_series &curve)
{
    ImPlotSpec spec;
    if(curve.mark == plot_mark::markers)
    {
        spec.Marker = ImPlotMarker_Circle;
        ImPlot::PlotScatter(curve.name.c_str(), curve.abscissa.data(), curve.ordinate.data(), drawn_length(curve), spec);
        return;
    }

    if(curve.mark == plot_mark::segments)
        spec.Flags = ImPlotLineFlags_Segments;
    ImPlot::PlotLine(curve.name.c_str(), curve.abscissa.data(), curve.ordinate.data(), drawn_length(curve), spec);
}

// The press also starts the plot's pan, so the pick waits for the release and a press that strayed
// past the GUI library's drag threshold stays a pan.
bool clicked(const ImGuiIO &io)
{
    return ImGui::IsMouseReleased(ImGuiMouseButton_Left) && io.MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] < io.MouseDragThreshold * io.MouseDragThreshold;
}

void report_pick(const plot_pick &pick, std::size_t index)
{
    if(!pick || !clicked(ImGui::GetIO()) || !ImGui::GetIO().KeyShift || !ImPlot::IsPlotHovered())
        return;

    const ImPlotPoint at = ImPlot::GetPlotMousePos();
    pick(index, at.x, at.y);
}

// The panel's title already names what is drawn, so each plot's own title is hidden; the index still
// stands in the identifier, because plots sharing one identifier are one plot to the library.
void render_frame(const plot_frame &shown, const std::string &abscissa_label, std::size_t index, float height, const plot_pick &pick)
{
    if(!ImPlot::BeginPlot(("##plot" + std::to_string(index)).c_str(), ImVec2(-1.f, height)))
        return;

    setup_axes(shown, abscissa_label);
    for(const plot_series &curve : shown.series)
        render_curve(curve);
    report_pick(pick, index);

    ImPlot::EndPlot();
}

void render_frames(const plot_reading &shown, const plot_pick &pick)
{
    if(shown.frames.empty())
        return;

    const float height = shared_height(shown.frames.size());
    for(std::size_t index = 0; index < shown.frames.size(); ++index)
        render_frame(shown.frames[index], shown.abscissa_label, index, height, pick);
}

}

plot_window::plot_window(std::string name, std::function<void()> controls, plot_source source)
        : plot_window(std::move(name), std::move(controls), std::move(source), plot_pick())
{
}

plot_window::plot_window(std::string name, std::function<void()> controls, plot_source source, plot_pick pick)
        : imgui_window(std::move(name))
        , m_pick(std::move(pick))
        , m_source(std::move(source))
        , m_controls(std::move(controls))
{
}

void plot_window::render()
{
    ImGui::Begin(display_name().c_str());

    if(m_controls)
        m_controls();

    const plot_reading shown = m_source ? m_source() : plot_reading{};
    if(!shown.message.empty())
        ImGui::TextUnformatted(shown.message.c_str());
    else if(ImPlot::GetCurrentContext() == nullptr)
        ImGui::TextUnformatted(absent_context_line);
    else
        render_frames(shown, m_pick);

    ImGui::End();
}

}
