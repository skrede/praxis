#include "plot_key.h"

#include "praxis/scene/plot_window.h"

#include <imgui.h>
#include <imgui_internal.h>

#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>

namespace praxis::scene {

namespace {

// The plotting library shows a series under the part of its name before any "##", and none whose
// name starts with one.
std::vector<std::string> shown_names(const std::vector<plot_series> &series)
{
    std::vector<std::string> names;
    for(const plot_series &curve : series)
        if(ImGui::FindRenderedTextEnd(curve.name.c_str()) != curve.name.c_str() && std::find(names.begin(), names.end(), curve.name) == names.end())
            names.push_back(curve.name);

    return names;
}

float entry_width(const std::string &name)
{
    return ImGui::GetTextLineHeight() + ImGui::GetStyle().ItemInnerSpacing.x + ImGui::CalcTextSize(name.c_str(), nullptr, true).x;
}

// Whether an entry continues the row that ends at `x` within `width`; `x` moves past the entry.
bool continues_row(float &x, float entry, float width)
{
    const float after = x + ImGui::GetStyle().ItemSpacing.x + entry;
    const bool fits   = x > 0.f && after <= width;
    x                 = fits ? after : entry;

    return fits;
}

void draw_entry(const std::string &name, ImU32 color)
{
    const float side    = ImGui::GetTextLineHeight();
    const ImVec2 corner = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddRectFilled(corner, ImVec2(corner.x + side, corner.y + side), color);
    ImGui::Dummy(ImVec2(side, side));
    ImGui::SameLine(0.f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::TextUnformatted(name.c_str(), ImGui::FindRenderedTextEnd(name.c_str()));
}

}

plot_key::plot_key(const std::vector<plot_series> &series, float width)
        : m_width(width)
        , m_names(shown_names(series))
        , m_colors(m_names.size(), 0u)
{
}

float plot_key::height() const
{
    return static_cast<float>(rows()) * (ImGui::GetTextLineHeight() + ImGui::GetStyle().ItemSpacing.y);
}

void plot_key::take_color(const std::string &name, ImVec4 color)
{
    const auto found = std::find(m_names.begin(), m_names.end(), name);
    if(found != m_names.end())
        m_colors[static_cast<std::size_t>(found - m_names.begin())] = ImGui::GetColorU32(color);
}

void plot_key::draw() const
{
    float x = 0.f;
    for(std::size_t at = 0; at < m_names.size(); ++at)
    {
        if(continues_row(x, entry_width(m_names[at]), m_width))
            ImGui::SameLine(0.f, ImGui::GetStyle().ItemSpacing.x);
        draw_entry(m_names[at], m_colors[at]);
    }
}

std::size_t plot_key::rows() const
{
    std::size_t counted = 0;
    float x             = 0.f;
    for(const std::string &name : m_names)
        if(!continues_row(x, entry_width(name), m_width))
            ++counted;

    return counted;
}

}
