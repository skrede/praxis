#ifndef HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_KEY_ROWS_H
#define HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_KEY_ROWS_H

#include "imgui_frame.h"

#include "praxis/scene/key_window.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <cmath>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <algorithm>

namespace praxis::fixture {

inline ImU32 swatch_of(std::uint32_t tone)
{
    return IM_COL32((tone >> 16u) & 0xffu, (tone >> 8u) & 0xffu, tone & 0xffu, 0xffu);
}

struct drawn_key
{
    std::vector<std::string> lines;
    std::vector<ImU32> swatches;
};

inline std::vector<std::string> logged_lines(const std::string &logged)
{
    std::vector<std::string> lines;
    for(std::size_t from = 0, to = 0; from < logged.size(); from = to + 1u)
    {
        to                       = std::min(logged.find('\n', from), logged.size());
        const std::string spoken = logged.substr(from, to - from);
        if(spoken.find_first_not_of(' ') != std::string::npos)
            lines.push_back(spoken.substr(spoken.find_first_not_of(' ')));
    }

    return lines;
}

// Top to bottom, the colors of the filled squares a text line high a window drew: four vertices of
// one color on one texel, corner by corner.
inline std::vector<ImU32> swatches_in(const std::string &window)
{
    const float side = ImGui::GetTextLineHeight();
    const auto near  = [](float a, float b) { return std::abs(a - b) <= 1.0e-3f; };
    const auto from  = [&near](const ImDrawVert &corner, const ImDrawVert &origin, float right, float down)
    {
        return corner.col == origin.col && corner.uv.x == origin.uv.x && corner.uv.y == origin.uv.y && near(corner.pos.x, origin.pos.x + right) &&
                near(corner.pos.y, origin.pos.y + down);
    };

    std::vector<ImU32> swatches;
    const ImVector<ImDrawVert> &corners = ImGui::FindWindowByName(window.c_str())->DrawList->VtxBuffer;
    for(int at = 0; at + 3 < corners.Size; ++at)
        if(from(corners[at + 1], corners[at], side, 0.f) && from(corners[at + 2], corners[at], side, side) && from(corners[at + 3], corners[at], 0.f, side))
            swatches.push_back(corners[at].col);

    return swatches;
}

// The key's text lines as logged after its title, and its swatches.
inline drawn_key drawn_rows(scene::key_window &key)
{
    tests::imgui_frame frames;
    std::string logged;
    frames.draw(
            [&key, &logged]
            {
                ImGui::SetNextWindowPos(ImVec2(0.f, 0.f));
                ImGui::SetNextWindowSize(ImVec2(400.f, 600.f));
                ImGui::LogToBuffer();
                key.render();
                logged = GImGui->LogBuffer.c_str();
                ImGui::LogFinish();
            });

    drawn_key drawn{logged_lines(logged), swatches_in(key.display_name())};
    REQUIRE(!drawn.lines.empty());
    CHECK(drawn.lines.front() == key.display_name());
    drawn.lines.erase(drawn.lines.begin());

    return drawn;
}

}

#endif
