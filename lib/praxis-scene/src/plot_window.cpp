#include "plot_key.h"
#include "plot_ring.h"

#include "praxis/scene/plot_window.h"

#include <imgui.h>
#include <implot.h>

#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <optional>
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

// The frames of one reading share what the panel leaves once their keys are set aside, less the
// spacing the GUI library puts between successive items. The plotting library raises a plot given
// the fill sentinel to its own minimum but not one given a height, so that floor is applied here.
float shared_height(std::size_t count, float keys)
{
    const float between = ImGui::GetStyle().ItemSpacing.y * static_cast<float>(count - 1u);
    const float share   = (ImGui::GetContentRegionAvail().y - between - keys) / static_cast<float>(count);

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

ImVec4 color_of(std::uint32_t tone)
{
    const auto channel = [tone](unsigned shift) { return static_cast<float>((tone >> shift) & 0xffu) / 255.f; };

    return ImVec4(channel(16u), channel(8u), channel(0u), 1.f);
}

void render_curve(const plot_series &curve)
{
    ImPlotSpec spec;
    if(curve.tone)
        spec.LineColor = color_of(*curve.tone);
    if(curve.mark == plot_mark::line || curve.mark == plot_mark::segments)
    {
        if(curve.mark == plot_mark::segments)
            spec.Flags = ImPlotLineFlags_Segments;
        ImPlot::PlotLine(curve.name.c_str(), curve.abscissa.data(), curve.ordinate.data(), drawn_length(curve), spec);
        return;
    }

    spec.Marker = ImPlotMarker_Circle;
    if(curve.mark == plot_mark::ring)
    {
        spec.MarkerFillColor = ImVec4(0.f, 0.f, 0.f, 0.f);
        spec.MarkerSize *= ring_radius_ratio;
        spec.LineWeight = ring_weight;
    }
    ImPlot::PlotScatter(curve.name.c_str(), curve.abscissa.data(), curve.ordinate.data(), drawn_length(curve), spec);
}

// A plot clipped out of view draws no items, so its key takes each series' tone, and the colormap's
// colors in order for the series carrying none.
void key_clipped(const plot_frame &shown, plot_key &key)
{
    int untoned = 0;
    for(const plot_series &curve : shown.series)
        key.take_color(curve.name, curve.tone ? color_of(*curve.tone) : ImPlot::GetColormapColor(untoned++));
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
void render_frame(const plot_frame &shown, const std::string &abscissa_label, std::size_t index, float height, const plot_pick &pick, std::optional<plot_key> &key)
{
    if(ImPlot::BeginPlot(("##plot" + std::to_string(index)).c_str(), ImVec2(-1.f, height), key ? ImPlotFlags_NoLegend : ImPlotFlags_None))
    {
        setup_axes(shown, abscissa_label);
        for(const plot_series &curve : shown.series)
        {
            render_curve(curve);
            if(key)
                key->take_color(curve.name, curve.tone ? color_of(*curve.tone) : ImPlot::GetLastItemColor());
        }
        report_pick(pick, index);

        ImPlot::EndPlot();
    }
    else if(key)
        key_clipped(shown, *key);

    if(key)
        key->draw();
}

void render_frames(const plot_reading &shown, const plot_pick &pick, plot_legend legend)
{
    if(shown.frames.empty())
        return;

    std::vector<std::optional<plot_key>> keys(shown.frames.size());
    float keyed = 0.f;
    for(std::size_t index = 0; index < keys.size() && legend == plot_legend::wrapped_key; ++index)
        keyed += keys[index].emplace(shown.frames[index].series, ImGui::GetContentRegionAvail().x).height();

    const float height = shared_height(shown.frames.size(), keyed);
    for(std::size_t index = 0; index < shown.frames.size(); ++index)
        render_frame(shown.frames[index], shown.abscissa_label, index, height, pick, keys[index]);
}

}

plot_window::plot_window(std::string name, std::function<void()> controls, plot_source source)
        : plot_window(std::move(name), std::move(controls), std::move(source), plot_pick())
{
}

plot_window::plot_window(std::string name, std::function<void()> controls, plot_source source, plot_pick pick)
        : imgui_window(std::move(name))
        , m_pick(std::move(pick))
        , m_legend(plot_legend::inside_north_west)
        , m_source(std::move(source))
        , m_controls(std::move(controls))
{
}

void plot_window::legend_at(plot_legend at)
{
    m_legend = at;
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
        render_frames(shown, m_pick, m_legend);

    ImGui::End();
}

}
