#include "imgui_frame.h"

#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <imgui.h>

#include <string>
#include <utility>

namespace {

using Catch::Matchers::WithinAbs;
using praxis::scene::window_corner;
using praxis::scene::opening_placement;

struct measured
{
    ImVec2 position;
    ImVec2 size;
    ImVec2 work_position;
    ImVec2 work_size;
    float unit;
};

class measuring_window : public praxis::scene::imgui_window
{
public:
    explicit measuring_window(std::string name)
            : imgui_window(std::move(name))
            , m_seen()
    {
    }

    void render() override
    {
        ImGui::Begin(display_name().c_str());
        ImGui::TextUnformatted("measured");
        const ImGuiViewport &viewport = *ImGui::GetMainViewport();
        m_seen                        = measured{ImGui::GetWindowPos(), ImGui::GetWindowSize(), viewport.WorkPos, viewport.WorkSize, ImGui::GetFontSize()};
        ImGui::End();
    }

    const measured &seen() const
    {
        return m_seen;
    }

private:
    measured m_seen;
};

measured opened(measuring_window &panel, const char *layout = nullptr)
{
    praxis::tests::imgui_frame frame;
    if(layout != nullptr)
        ImGui::LoadIniSettingsFromMemory(layout);
    frame.draw([&panel] { praxis::scene::render_placed(panel); });

    return panel.seen();
}

}

TEST_CASE("a window given an opening placement opens there, inset from its corner in font sizes", "[scene]")
{
    measuring_window panel("Placed top right");
    panel.open_at(opening_placement{window_corner::top_right, 1.f, 2.f, 20.f, 10.f});

    const measured seen = opened(panel);

    CHECK_THAT(seen.position.x + seen.size.x, WithinAbs(seen.work_position.x + seen.work_size.x - seen.unit, 0.5));
    CHECK_THAT(seen.position.y, WithinAbs(seen.work_position.y + 2.f * seen.unit, 0.5));
    CHECK_THAT(seen.size.x, WithinAbs(20.f * seen.unit, 0.5));
    CHECK_THAT(seen.size.y, WithinAbs(10.f * seen.unit, 0.5));
}

TEST_CASE("a placement at a bottom corner is inset upward from the work area's bottom", "[scene]")
{
    measuring_window panel("Placed bottom left");
    panel.open_at(opening_placement{window_corner::bottom_left, 3.f, 4.f, 10.f, 5.f});

    const measured seen = opened(panel);

    CHECK_THAT(seen.position.x, WithinAbs(seen.work_position.x + 3.f * seen.unit, 0.5));
    CHECK_THAT(seen.position.y + seen.size.y, WithinAbs(seen.work_position.y + seen.work_size.y - 4.f * seen.unit, 0.5));
}

TEST_CASE("a layout entry for the window's title wins over its opening placement", "[scene]")
{
    measuring_window panel("Placed and saved");
    panel.open_at(opening_placement{window_corner::top_right, 1.f, 2.f, 20.f, 10.f});

    const measured seen = opened(panel, "[Window][Placed and saved]\nPos=100,120\nSize=300,200\n");

    CHECK_THAT(seen.position.x, WithinAbs(100.0, 0.5));
    CHECK_THAT(seen.position.y, WithinAbs(120.0, 0.5));
    CHECK_THAT(seen.size.x, WithinAbs(300.0, 0.5));
    CHECK_THAT(seen.size.y, WithinAbs(200.0, 0.5));
}

TEST_CASE("a window given no placement carries none and opens where the library alone opens it", "[scene]")
{
    measuring_window placed_through("Unplaced");
    measuring_window drawn_alone("Unplaced");
    REQUIRE_FALSE(placed_through.opening().has_value());

    const measured through = opened(placed_through);
    measured alone{};
    {
        praxis::tests::imgui_frame frame;
        frame.draw([&drawn_alone] { drawn_alone.render(); });
        alone = drawn_alone.seen();
    }

    CHECK(through.position.x == alone.position.x);
    CHECK(through.position.y == alone.position.y);
    CHECK(through.size.x == alone.size.x);
    CHECK(through.size.y == alone.size.y);
}

TEST_CASE("a window reports the placement it was given", "[scene]")
{
    measuring_window panel("Reported");
    panel.open_at(opening_placement{window_corner::bottom_right, 1.5f, 2.5f, 30.f, 12.f});

    REQUIRE(panel.opening().has_value());
    CHECK(panel.opening()->corner == window_corner::bottom_right);
    CHECK(panel.opening()->width == 30.f);
}
