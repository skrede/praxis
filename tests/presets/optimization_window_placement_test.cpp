#include "imgui_frame.h"
#include "window_placements.h"
#include "scenario_placements.h"
#include "composed_optimization.h"

#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>

using namespace praxis;
using namespace praxis::fixture;

namespace {

using scene::opening_placement;

template<std::size_t count>
void check_pinned(const composed_optimization &composed, const std::array<pinned_placement, count> &pinned, std::size_t dimension)
{
    REQUIRE(composed.preset->windows.size() == count);
    for(std::size_t at = 0; at < count; ++at)
    {
        const std::shared_ptr<scene::imgui_window> &window = composed.preset->windows[at];
        INFO(window->display_name());
        REQUIRE(window->display_name() == pinned[at].name);
        REQUIRE(window->opening().has_value());
        CHECK(same(*window->opening(), pinned[at].at));

        const std::optional<opening_placement> answered = presets::scenario_window_placement(pinned[at].title, dimension);
        REQUIRE(answered.has_value());
        CHECK(same(*window->opening(), *answered));
    }
}

struct named_rectangle
{
    std::string name;
    scene::window_rectangle at;
};

bool spans_cross(float start, float extent, float other_start, float other_extent)
{
    return start < other_start + other_extent && other_start < start + extent;
}

// Sharing an edge is not an overlap.
bool overlap(const scene::window_rectangle &one, const scene::window_rectangle &other)
{
    return spans_cross(one.position.x, one.size.x, other.position.x, other.size.x) && spans_cross(one.position.y, one.size.y, other.position.y, other.size.y);
}

std::vector<named_rectangle> opened_in(const composed_optimization &composed, const ImGuiViewport &viewport)
{
    REQUIRE(ImGui::GetFontSize() == 13.f);
    REQUIRE(viewport.WorkSize.x == 1280.f);
    REQUIRE(viewport.WorkSize.y == 720.f);

    std::vector<named_rectangle> opened;
    for(const std::shared_ptr<scene::imgui_window> &window : composed.preset->windows)
    {
        REQUIRE(window->opening().has_value());
        opened.push_back({window->display_name(), scene::opening_rectangle(*window->opening(), viewport.WorkPos, viewport.WorkSize, ImGui::GetFontSize())});
    }

    return opened;
}

// The scenario's windows where they open first at 1280 x 720 and font scale 1 in a context with no
// layout file.
std::vector<named_rectangle> opened_at_smallest_screen(const composed_optimization &composed)
{
    tests::imgui_frame frame;
    ImGui::GetIO().DisplaySize = ImVec2(1280.f, 720.f);

    std::vector<named_rectangle> opened;
    frame.draw_frame([&composed, &opened] { opened = opened_in(composed, *ImGui::GetMainViewport()); });
    REQUIRE(frame.has_draw_data());

    return opened;
}

void check_apart(const std::vector<named_rectangle> &opened)
{
    REQUIRE(opened.size() > 1u);
    for(std::size_t one = 0; one < opened.size(); ++one)
        for(std::size_t other = one + 1; other < opened.size(); ++other)
        {
            INFO(opened[one].name << " and " << opened[other].name);
            CHECK_FALSE(overlap(opened[one].at, opened[other].at));
        }
}

}

TEST_CASE("every window the curve composes opens at the placement answered for its title", "[presets]")
{
    const composed_optimization composed(placed_document("curve.xml", placed_curve_document), {}, "placed curve");

    check_pinned(composed, curve_placements, 1u);
}

TEST_CASE("every window the surface composes opens at the placement answered for its title", "[presets]")
{
    const composed_optimization composed(placed_document("surface.xml", placed_surface_document), {}, "placed surface");

    check_pinned(composed, surface_placements, 2u);
}

TEST_CASE("a title the scenario composes no window under is answered no placement", "[presets]")
{
    CHECK_FALSE(presets::scenario_window_placement("Graph", 2u).has_value());
    CHECK_FALSE(presets::scenario_window_placement("Presets", 1u).has_value());
}

TEST_CASE("the curve's windows open apart from each other at 1280 x 720", "[presets]")
{
    const composed_optimization composed(placed_document("curve.xml", placed_curve_document), {}, "placed curve");

    check_apart(opened_at_smallest_screen(composed));
}

TEST_CASE("the surface's windows open apart from each other at 1280 x 720", "[presets]")
{
    const composed_optimization composed(placed_document("surface.xml", placed_surface_document), {}, "placed surface");

    check_apart(opened_at_smallest_screen(composed));
}
