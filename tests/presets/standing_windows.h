#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_STANDING_WINDOWS_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_STANDING_WINDOWS_H

#include <imgui.h>

#include <span>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

namespace praxis::fixture {

struct standing_window
{
    std::string name;
    ImVec2 position;
    ImVec2 size;
};

namespace detail {

inline float framed_width(const char *label)
{
    return ImGui::GetStyle().FramePadding.x * 2.f + ImGui::CalcTextSize(label).x;
}

// Fit to one row, the widest name beside the combo's arrow square then Load, Unload and Save, under
// the title bar, at the GUI library's default first place; that library truncates a size to whole px.
inline standing_window preset_selector(std::span<const std::string> presets)
{
    const ImGuiStyle &style = ImGui::GetStyle();
    float widest            = 0.f;
    for(const std::string &name : presets)
        widest = std::max(widest, framed_width(name.c_str()));

    const float spacing = style.ItemSpacing.x;
    const float row     = widest + ImGui::GetFrameHeight() + spacing + framed_width("Load") + spacing + framed_width("Unload") + spacing + framed_width("Save");
    const ImVec2 origin = ImGui::GetMainViewport()->Pos;

    return {"Presets", ImVec2(origin.x + 60.f, origin.y + 60.f),
            ImVec2(std::trunc(2.f * style.WindowPadding.x + row), std::trunc(2.f * ImGui::GetFrameHeight() + 2.f * style.WindowPadding.y))};
}

}

// The display and the style the demonstration gives its GUI at font scale `scale`.
inline void style_as_shown(ImVec2 display, float scale)
{
    ImGui::GetIO().DisplaySize = display;
    ImGuiStyle &style          = ImGui::GetStyle();
    style                      = ImGuiStyle();
    style.FontScaleDpi         = scale;
    style.ScaleAllSizes(scale);
}

// Where the windows the visualizer stands beside every preset open first in the current GUI context,
// the preset selector offering `presets`. s is the font size over 13 px.
inline std::vector<standing_window> standing_windows(std::span<const std::string> presets)
{
    const ImGuiViewport &viewport = *ImGui::GetMainViewport();
    const ImVec2 at               = viewport.WorkPos;
    const ImVec2 area             = viewport.WorkSize;
    const ImVec2 panel(0.45f * area.x, 0.25f * area.y);
    const float s = ImGui::GetFontSize() / 13.f;

    return {{"Messages", ImVec2(at.x + 20.f, at.y + area.y - panel.y - 20.f), panel},
            {"Stepped work", ImVec2(at.x + area.x - panel.x - 20.f, at.y + 20.f), panel},
            {"##ViewGizmo", ImVec2(at.x + area.x - 12.f * s - 96.f * s, at.y + 12.f * s), ImVec2(96.f * s, 96.f * s)},
            detail::preset_selector(presets)};
}

}

#endif
