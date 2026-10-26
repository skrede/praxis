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
#include <utility>
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
    std::size_t outlined;
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

// Whether a stroke in the text color passes each corner of the square at `origin`: a vertex of that
// color within 2.5 px of the corner, other than the corner itself, which a fill of that color puts there.
inline bool outlined(const ImVector<ImDrawVert> &drawn, ImVec2 origin, float side)
{
    const ImU32 edge = ImGui::GetColorU32(ImGuiCol_Text);
    const auto at    = [&drawn, edge](float x, float y)
    {
        return std::any_of(drawn.begin(), drawn.end(),
                           [edge, x, y](const ImDrawVert &v)
                           {
                               const float apart = std::max(std::abs(v.pos.x - x), std::abs(v.pos.y - y));
                               return v.col == edge && apart > 0.25f && apart <= 2.5f;
                           });
    };

    return at(origin.x, origin.y) && at(origin.x + side, origin.y) && at(origin.x + side, origin.y + side) && at(origin.x, origin.y + side);
}

struct drawn_swatch
{
    ImU32 fill;
    ImVec2 origin;
};

// Top to bottom, the colors and the top-left corners of the filled squares a text line high a window
// drew: four vertices of one color on one texel, corner by corner.
inline std::vector<drawn_swatch> swatches_in(const std::string &window)
{
    const float side = ImGui::GetTextLineHeight();
    const auto near  = [](float a, float b) { return std::abs(a - b) <= 1.0e-3f; };
    const auto from  = [&near](const ImDrawVert &corner, const ImDrawVert &origin, float right, float down)
    {
        return corner.col == origin.col && corner.uv.x == origin.uv.x && corner.uv.y == origin.uv.y && near(corner.pos.x, origin.pos.x + right) &&
                near(corner.pos.y, origin.pos.y + down);
    };

    std::vector<drawn_swatch> swatches;
    const ImVector<ImDrawVert> &corners = ImGui::FindWindowByName(window.c_str())->DrawList->VtxBuffer;
    for(int at = 0; at + 3 < corners.Size; ++at)
        if(from(corners[at + 1], corners[at], side, 0.f) && from(corners[at + 2], corners[at], side, side) && from(corners[at + 3], corners[at], 0.f, side))
            swatches.push_back(drawn_swatch{corners[at].col, corners[at].pos});

    return swatches;
}

// A window's text lines beside the fills of its swatches, top to bottom, and how many of them a stroke
// in the text color frames.
inline drawn_key keyed_swatches(const std::string &window, std::vector<std::string> lines)
{
    drawn_key drawn{std::move(lines), {}, 0u};
    const ImVector<ImDrawVert> &vertices = ImGui::FindWindowByName(window.c_str())->DrawList->VtxBuffer;
    for(const drawn_swatch &swatch : swatches_in(window))
    {
        drawn.swatches.push_back(swatch.fill);
        drawn.outlined += outlined(vertices, swatch.origin, ImGui::GetTextLineHeight()) ? 1u : 0u;
    }

    return drawn;
}

// The key's text lines as logged after its title, its swatches, and how many of them are outlined.
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

    drawn_key drawn = keyed_swatches(key.display_name(), logged_lines(logged));
    REQUIRE(!drawn.lines.empty());
    CHECK(drawn.lines.front() == key.display_name());
    drawn.lines.erase(drawn.lines.begin());

    return drawn;
}

}

#endif
