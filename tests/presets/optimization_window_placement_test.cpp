#include "imgui_frame.h"
#include "scratch_directory.h"
#include "window_placements.h"
#include "composed_optimization.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/store.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <utility>
#include <optional>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::fixture;

namespace {

using scene::window_corner;
using scene::opening_placement;

constexpr const char *curve_document = "<optimization><preset name=\"placed curve\" scenario=\"curve\"/><function source=\"manual\" expression=\"x^3 - 2x - 5\"/>"
                                       "<range x_lower=\"-3\" x_upper=\"3\"/><seed x=\"1\"/></optimization>";

constexpr const char *surface_document = "<optimization><preset name=\"placed surface\" scenario=\"surface\"/><function source=\"manual\" expression=\"x^2 + y^2 - 1\"/>"
                                         "<range x_lower=\"-2\" x_upper=\"2\" y_lower=\"-2\" y_upper=\"2\"/><seed x=\"-2\" y=\"-2\"/></optimization>";

struct pinned_placement
{
    std::string_view title;
    opening_placement at;
};

constexpr std::array<pinned_placement, 5> curve_placements{{
        {"Function", {window_corner::top_left, 1.f, 10.f, 35.f, 15.f}},
        {"Graph", {window_corner::top_left, 37.f, 17.f, 20.f, 22.f}},
        {"Display", {window_corner::top_left, 37.f, 1.f, 12.f, 10.f}},
        {"Solve", {window_corner::bottom_right, 1.f, 20.f, 36.f, 19.f}},
        {"Linearization", {window_corner::bottom_right, 1.f, 1.f, 39.f, 14.f}},
}};

constexpr std::array<pinned_placement, 4> surface_placements{{
        {"Function", {window_corner::top_left, 1.f, 10.f, 35.f, 19.f}},
        {"Display", {window_corner::top_left, 37.f, 1.f, 12.f, 15.f}},
        {"Solve", {window_corner::bottom_right, 1.f, 20.f, 36.f, 19.f}},
        {"Linearization", {window_corner::bottom_right, 1.f, 1.f, 39.f, 18.f}},
}};

config::location written(const std::string &file, const char *body)
{
    const std::filesystem::path directory = shared_scratch_directory() / "optimization_window_placement";
    std::filesystem::create_directories(directory);
    std::ofstream(directory / file, std::ios::binary | std::ios::trunc) << body << "\n";

    return config::resolve(file, directory);
}

bool same(const opening_placement &one, const opening_placement &other)
{
    return one.corner == other.corner && one.inset_x == other.inset_x && one.inset_y == other.inset_y && one.width == other.width && one.height == other.height;
}

template<std::size_t count>
void check_pinned(const composed_optimization &composed, const std::array<pinned_placement, count> &pinned, std::size_t dimension)
{
    REQUIRE(composed.preset->windows.size() == count);
    for(std::size_t at = 0; at < count; ++at)
    {
        const std::shared_ptr<scene::imgui_window> &window = composed.preset->windows[at];
        INFO(window->display_name());
        REQUIRE(window->display_name() == pinned[at].title);
        REQUIRE(window->opening().has_value());
        CHECK(same(*window->opening(), pinned[at].at));

        const std::optional<opening_placement> answered = presets::scenario_window_placement(window->display_name(), dimension);
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

// Where the windows every demonstration stands open without a layout file, at font scale 1; the
// preset selector at the library's default place, as large as its row.
std::vector<named_rectangle> standing_windows(ImVec2 work_position, ImVec2 work_size)
{
    const ImVec2 quarter(work_size.x * 0.45f, work_size.y * 0.25f);

    return {
            {"message log", {ImVec2(work_position.x + 20.f, work_position.y + work_size.y - quarter.y - 20.f), quarter}},
            {"stepped work", {ImVec2(work_position.x + work_size.x - quarter.x - 20.f, work_position.y + 20.f), quarter}},
            {"view gizmo", {ImVec2(work_position.x + work_size.x - 12.f - 96.f, work_position.y + 12.f), ImVec2(96.f, 96.f)}},
            {"preset selector", {ImVec2(60.f, 60.f), ImVec2(182.f, 54.f)}},
    };
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
    for(named_rectangle &standing : standing_windows(viewport.WorkPos, viewport.WorkSize))
        opened.push_back(std::move(standing));

    return opened;
}

// The scenario's windows where they open first at 1280 x 720 and font scale 1 in a context with no
// layout file, the standing windows after them.
std::vector<named_rectangle> opened_at_smallest_screen(const composed_optimization &composed)
{
    tests::imgui_frame frame;
    ImGui::GetIO().DisplaySize = ImVec2(1280.f, 720.f);

    std::vector<named_rectangle> opened;
    frame.draw_frame([&composed, &opened] { opened = opened_in(composed, *ImGui::GetMainViewport()); });
    REQUIRE(frame.has_draw_data());

    return opened;
}

void check_apart(const std::vector<named_rectangle> &opened, std::size_t scenario_windows)
{
    REQUIRE(opened.size() > scenario_windows);
    for(std::size_t one = 0; one < scenario_windows; ++one)
        for(std::size_t other = one + 1; other < opened.size(); ++other)
        {
            INFO(opened[one].name << " and " << opened[other].name);
            CHECK_FALSE(overlap(opened[one].at, opened[other].at));
        }
}

}

TEST_CASE("every window the curve composes opens at the placement answered for its title", "[presets]")
{
    const composed_optimization composed(written("curve.xml", curve_document), {}, "placed curve");

    check_pinned(composed, curve_placements, 1u);
}

TEST_CASE("every window the surface composes opens at the placement answered for its title", "[presets]")
{
    const composed_optimization composed(written("surface.xml", surface_document), {}, "placed surface");

    check_pinned(composed, surface_placements, 2u);
}

TEST_CASE("a title the scenario composes no window under is answered no placement", "[presets]")
{
    CHECK_FALSE(presets::scenario_window_placement("Graph", 2u).has_value());
    CHECK_FALSE(presets::scenario_window_placement("Presets", 1u).has_value());
}

TEST_CASE("the curve's windows open apart from each other and from the standing windows at 1280 x 720", "[presets]")
{
    const composed_optimization composed(written("curve.xml", curve_document), {}, "placed curve");

    check_apart(opened_at_smallest_screen(composed), composed.preset->windows.size());
}

TEST_CASE("the surface's windows open apart from each other and from the standing windows at 1280 x 720", "[presets]")
{
    const composed_optimization composed(written("surface.xml", surface_document), {}, "placed surface");

    check_apart(opened_at_smallest_screen(composed), composed.preset->windows.size());
}
