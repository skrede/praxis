#include "praxis/scene/imgui_window.h"

#include <imgui.h>

#include <optional>

namespace praxis::scene {

namespace {

bool on_right(window_corner corner)
{
    return corner == window_corner::top_right || corner == window_corner::bottom_right;
}

bool on_bottom(window_corner corner)
{
    return corner == window_corner::bottom_left || corner == window_corner::bottom_right;
}

float from_edge(float start, float extent, float inset, float size, bool far)
{
    return far ? start + extent - inset - size : start + inset;
}

}

window_rectangle opening_rectangle(const opening_placement &at, ImVec2 work_position, ImVec2 work_size, float font_size)
{
    const ImVec2 size(at.width * font_size, at.height * font_size);
    const float x = from_edge(work_position.x, work_size.x, at.inset_x * font_size, size.x, on_right(at.corner));
    const float y = from_edge(work_position.y, work_size.y, at.inset_y * font_size, size.y, on_bottom(at.corner));

    return window_rectangle{ImVec2(x, y), size};
}

void imgui_window::open_at(opening_placement at)
{
    m_opening = at;
}

const std::optional<opening_placement> &imgui_window::opening() const
{
    return m_opening;
}

void render_placed(imgui_window &window)
{
    if(window.opening())
    {
        const ImGuiViewport &viewport = *ImGui::GetMainViewport();
        const window_rectangle opened = opening_rectangle(*window.opening(), viewport.WorkPos, viewport.WorkSize, ImGui::GetFontSize());
        ImGui::SetNextWindowPos(opened.position, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(opened.size, ImGuiCond_FirstUseEver);
    }
    window.render();
}

}
