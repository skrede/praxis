#include "scenario_placements.h"
#include "composed_optimization.h"

#include "imgui_frame.h"

#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <array>
#include <memory>
#include <string>
#include <cstddef>
#include <fstream>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr const char *bare_title_layout = "[Window][Function]\nPos=600,600\nSize=100,100\nCollapsed=0\n\n"
                                          "[Window][Display]\nPos=700,600\nSize=100,100\nCollapsed=0\n\n"
                                          "[Window][Solve]\nPos=800,600\nSize=100,100\nCollapsed=0\n\n"
                                          "[Window][Linearization]\nPos=900,600\nSize=100,100\nCollapsed=0\n\n";

// Each window drawn through its opening placement, as the demonstration's window context draws it.
void drawn_placed(tests::imgui_frame &frames, const composed_optimization &composed)
{
    frames.draw(
            [&composed]
            {
                for(const std::shared_ptr<scene::imgui_window> &window : composed.preset->windows)
                    scene::render_placed(*window);
            },
            4);
}

void check_rectangle(const ImGuiWindow &held, const scene::window_rectangle &wanted)
{
    CHECK(held.Pos.x == wanted.position.x);
    CHECK(held.Pos.y == wanted.position.y);
    CHECK(held.Size.x == wanted.size.x);
    CHECK(held.Size.y == wanted.size.y);
}

// Where the GUI library holds each window once drawn, against the placement pinned for it, and the
// title the window shows.
template<std::size_t count>
void check_applied(const composed_optimization &composed, const std::array<pinned_placement, count> &pinned)
{
    const ImGuiViewport &viewport = *ImGui::GetMainViewport();

    REQUIRE(composed.preset->windows.size() == count);
    for(std::size_t at = 0; at < count; ++at)
    {
        const std::string &name = composed.preset->windows[at]->display_name();
        INFO(name);
        const ImGuiWindow *held = ImGui::FindWindowByName(name.c_str());
        REQUIRE(held != nullptr);
        check_rectangle(*held, scene::opening_rectangle(pinned[at].at, viewport.WorkPos, viewport.WorkSize, ImGui::GetFontSize()));

        CHECK(name == pinned[at].name);
        CHECK(std::string_view(name.c_str(), ImGui::FindRenderedTextEnd(name.c_str())) == pinned[at].title);
    }
}

}

TEST_CASE("a surface opened after a curve in one context opens every window at the surface's placement", "[presets]")
{
    tests::imgui_frame frames;
    {
        const composed_optimization curve(placed_document("curve.xml", placed_curve_document), {}, "placed curve");
        drawn_placed(frames, curve);
        check_applied(curve, curve_placements);
    }

    const composed_optimization surface(placed_document("surface.xml", placed_surface_document), {}, "placed surface");
    drawn_placed(frames, surface);
    check_applied(surface, surface_placements);
}

TEST_CASE("a curve opened after a surface in one context opens every window at the curve's placement", "[presets]")
{
    tests::imgui_frame frames;
    {
        const composed_optimization surface(placed_document("surface.xml", placed_surface_document), {}, "placed surface");
        drawn_placed(frames, surface);
        check_applied(surface, surface_placements);
    }

    const composed_optimization curve(placed_document("curve.xml", placed_curve_document), {}, "placed curve");
    drawn_placed(frames, curve);
    check_applied(curve, curve_placements);
}

TEST_CASE("a layout entry held under a window's shown title alone moves none of the scenario's windows", "[presets]")
{
    const std::filesystem::path layout = shared_scratch_directory() / "bare_title_layout.ini";
    std::ofstream(layout, std::ios::binary | std::ios::trunc) << bare_title_layout;

    tests::imgui_frame frames(layout);
    const composed_optimization surface(placed_document("surface.xml", placed_surface_document), {}, "placed surface");
    drawn_placed(frames, surface);
    check_applied(surface, surface_placements);
}
