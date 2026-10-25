#include "praxis/scene/key_window.h"

#include <imgui.h>

#include <string>
#include <vector>
#include <cstdint>
#include <utility>

namespace praxis::scene {

namespace {

ImU32 opaque(std::uint32_t tone)
{
    return IM_COL32((tone >> 16u) & 0xffu, (tone >> 8u) & 0xffu, tone & 0xffu, 0xffu);
}

void draw_entry(const key_entry &entry)
{
    const float side    = ImGui::GetTextLineHeight();
    const ImVec2 corner = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddRectFilled(corner, ImVec2(corner.x + side, corner.y + side), opaque(entry.tone));
    ImGui::Dummy(ImVec2(side, side));
    ImGui::SameLine(0.f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::TextUnformatted(entry.label.c_str());
}

}

key_window::key_window(std::string name, key_source source)
        : imgui_window(std::move(name))
        , m_source(std::move(source))
{
}

void key_window::render()
{
    ImGui::Begin(display_name().c_str());

    const std::vector<key_entry> entries = m_source ? m_source() : std::vector<key_entry>{};
    for(const key_entry &entry : entries)
        draw_entry(entry);

    ImGui::End();
}

}
