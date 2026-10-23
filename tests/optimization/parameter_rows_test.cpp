#include "rows_stage.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/optimization/scenario_snapshot.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <memory>
#include <string>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr std::size_t letter_b   = 'b';
constexpr const char *cubic_text = "2x^3 + 5b";

// The minimum and the maximum share the line below the slider.
void type_range_end(tests::imgui_frame &frames, const drawing &draw, const char *end, const std::string &typed)
{
    stand_below_top(frames, draw, 1u);
    stand_along(frames, draw, control_id("Rows", letter_b, end), "the " + std::string(end) + " of 'b'");

    tap(frames, draw, ImGuiKey_Space);
    for(const char one : typed)
        ImGui::GetIO().AddInputCharacter(static_cast<unsigned int>(one));
    frames.draw_frame(draw);
    tap(frames, draw, ImGuiKey_Enter);
}

}

TEST_CASE("a slider being moved asks for its value before it is let go, one value queued at a time", "[optimization][function][manual]")
{
    rows_stage staged(cubic_text);
    tests::imgui_frame frames;
    frames.draw(staged.draw());

    stand_on(frames, staged.draw(), "Rows", letter_b, "b");
    tap(frames, staged.draw(), ImGuiKey_Space);
    tap(frames, staged.draw(), ImGuiKey_RightArrow);
    const double first = staged.rows.held().front().value;
    for(int step = 0; step < 3; ++step)
        tap(frames, staged.draw(), ImGuiKey_RightArrow);
    REQUIRE(first > 1.0);
    REQUIRE(staged.rows.held().front().value > first);

    CHECK(staged.published('b').value == first);
    frames.draw_frame(staged.draw());
    CHECK(staged.published('b').value == staged.rows.held().front().value);
}

TEST_CASE("a burst of slider moves queues one parameter edit and the value the slider rests at follows it", "[optimization][function][manual]")
{
    rows_stage staged(cubic_text);
    tests::imgui_frame frames;
    frames.draw(staged.draw());
    REQUIRE(staged.rows.held().size() == 1u);
    REQUIRE(staged.rows.held().front().value == 1.0);

    slide(frames, staged.draw(), 'b', 3u);
    slide(frames, staged.draw(), 'b', 4u);
    const double resting = staged.rows.held().front().value;
    REQUIRE(resting > 1.0);
    CHECK(staged.published('b').value != resting);

    frames.draw_frame(staged.draw());
    CHECK(staged.published('b').value == resting);
    frames.draw(staged.draw());
    CHECK(staged.published('b').value == resting);
    CHECK(staged.rows.held().front().value == resting);
}

TEST_CASE("a parameter's maximum is asked for once its field is left, and a range whose minimum is not below it is not", "[optimization][function][manual]")
{
    rows_stage staged(cubic_text);
    tests::imgui_frame frames;
    frames.draw(staged.draw());

    type_range_end(frames, staged.draw(), "max", "2.5");

    std::shared_ptr<const scenario_snapshot> shown = staged.stage.shown();
    REQUIRE(shown->parameters.size() == 1u);
    CHECK((shown->parameters.front().range.lower == -5.0 && shown->parameters.front().range.upper == 2.5));

    type_range_end(frames, staged.draw(), "min", "3");
    CHECK(staged.stage.shown() == shown);
}
