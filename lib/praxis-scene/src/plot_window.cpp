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

// The panel's title already names what is drawn, so each plot's own title is hidden; the index still
// stands in the identifier, because plots sharing one identifier are one plot to the library.
void render_frame(const plot_frame &shown, const std::string &abscissa_label, std::size_t index, float height)
{
    if(!ImPlot::BeginPlot(("##plot" + std::to_string(index)).c_str(), ImVec2(-1.f, height)))
        return;

    ImPlot::SetupAxes(abscissa_label.c_str(), shown.ordinate_label.c_str());
    if(shown.logarithmic_ordinate)
        ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
    ImPlot::SetupLegend(ImPlotLocation_NorthWest);

    for(const plot_series &curve : shown.series)
        ImPlot::PlotLine(curve.name.c_str(), curve.abscissa.data(), curve.ordinate.data(), drawn_length(curve));

    ImPlot::EndPlot();
}

void render_frames(const plot_reading &shown)
{
    if(shown.frames.empty())
        return;

    const float height = shared_height(shown.frames.size());
    for(std::size_t index = 0; index < shown.frames.size(); ++index)
        render_frame(shown.frames[index], shown.abscissa_label, index, height);
}

}

plot_window::plot_window(std::string name, std::function<void()> controls, plot_source source)
        : imgui_window(std::move(name))
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
        render_frames(shown);

    ImGui::End();
}

}
