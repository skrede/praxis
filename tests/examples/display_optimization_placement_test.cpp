#include "scratch_documents.h"
#include "offered_optimization.h"

#include "demo_write_back.h"

#include "praxis/scene/visualizer.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <string_view>

using namespace praxis;
using namespace scratch_documents;
using namespace offered_optimization;

namespace {

struct drawn_window
{
    std::string name;
    ImVec2 position;
    ImVec2 size;
};

constexpr ImGuiWindowFlags nested = ImGuiWindowFlags_ChildWindow | ImGuiWindowFlags_Tooltip | ImGuiWindowFlags_Popup;

// Every top-level window the last frame drew, where the GUI library holds it.
std::vector<drawn_window> drawn_windows()
{
    std::vector<drawn_window> drawn;
    for(const ImGuiWindow *window : GImGui->Windows)
        if(window->Active && !window->IsFallbackWindow && (window->Flags & nested) == 0)
            drawn.push_back({window->Name, window->Pos, window->Size});

    return drawn;
}

void draw_frame(scene::visualizer &view, scheduler::scheduler &loop)
{
    bool drawn = false;
    REQUIRE(view.executor().post([&view, &drawn] { drawn = view.render_once(); }).has_value());
    REQUIRE(loop.drain().has_value());
    REQUIRE(drawn);
}

// The windows a shipped preset's first frames put on screen in the demonstration's own arrangement:
// its registration, its save route and a 1280 x 720 window at font scale 1 with no layout file.
std::vector<drawn_window> opened_first(const std::string &named)
{
    const scratch where("display-optimization-placement");
    const offered shipped(where);
    scheduler::scheduler loop(scheduler::inline_workers);
    scene::visualizer view(shipped.registry, loop,
                           {.view = scene::visualizer::projection::perspective, .messages = nullptr, .window = {.width = 1280, .height = 720, .x = {}, .y = {}}, .root = where.state()});
    demo::install_write_back(view, shipped.writing);
    view.load_preset(named);
    REQUIRE(loop.drain().has_value());
    for(int frame = 0; frame < 4; ++frame)
        draw_frame(view, loop);

    REQUIRE(ImGui::GetFontSize() == 13.f);
    REQUIRE(ImGui::GetMainViewport()->WorkSize.x == 1280.f);
    REQUIRE(ImGui::GetMainViewport()->WorkSize.y == 720.f);
    return drawn_windows();
}

bool spans_cross(float start, float extent, float other_start, float other_extent)
{
    return start < other_start + other_extent && other_start < start + extent;
}

// Sharing an edge is not an overlap.
bool overlap(const drawn_window &one, const drawn_window &other)
{
    return spans_cross(one.position.x, one.size.x, other.position.x, other.size.x) && spans_cross(one.position.y, one.size.y, other.position.y, other.size.y);
}

bool composed_by_the_scenario(const drawn_window &window)
{
    return window.name.ends_with("##curve") || window.name.ends_with("##surface");
}

void check_apart(const std::vector<drawn_window> &drawn)
{
    for(const std::string_view standing : {"Messages", "Presets", "Stepped work", "##ViewGizmo"})
        CHECK(std::ranges::any_of(drawn, [standing](const drawn_window &window) { return window.name == standing; }));
    for(const drawn_window &one : drawn)
        for(const drawn_window &other : drawn)
            if(composed_by_the_scenario(one) && &one != &other)
            {
                INFO(one.name << " at " << one.position.x << ", " << one.position.y << " sized " << one.size.x << " x " << one.size.y << " and " << other.name << " at "
                              << other.position.x << ", " << other.position.y << " sized " << other.size.x << " x " << other.size.y);
                CHECK_FALSE(overlap(one, other));
            }
}

}

TEST_CASE("every shipped optimization preset opens its windows apart from each other and from the standing windows at 1280 x 720", "[examples][optimization][display]")
{
    const scratch where("display-optimization-names");
    const std::vector<std::string> names = offered(where).names;
    REQUIRE(names.size() == 6u);

    for(const std::string &named : names)
    {
        INFO(named);
        const std::vector<drawn_window> drawn = opened_first(named);
        const auto curve                      = std::ranges::count_if(drawn, [](const drawn_window &window) { return window.name.ends_with("##curve"); });
        const auto surface                    = std::ranges::count_if(drawn, [](const drawn_window &window) { return window.name.ends_with("##surface"); });
        CHECK(((curve == 5 && surface == 0) || (curve == 0 && surface == 4)));
        check_apart(drawn);
    }
}
