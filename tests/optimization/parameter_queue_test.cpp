#include "rows_stage.h"
#include "scenario_defaults.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "captured_log.h"
#include "panel_labels.h"

#include "praxis/optimization/scenario_state.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <string>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr const char *two_letters = "ax^2 + b";

// Enter opens a slider's value for typing, as a click holding Ctrl does.
void type_into_letter(tests::imgui_frame &frames, const drawing &draw, char letter, const char *typed)
{
    const char label[2] = {letter, '\0'};
    stand_on(frames, draw, "Rows", static_cast<std::size_t>(letter), label);
    tap(frames, draw, ImGuiKey_Enter);
    for(const char *at = typed; *at != 0; ++at)
        ImGui::GetIO().AddInputCharacter(static_cast<unsigned int>(*at));
    frames.draw_frame(draw);
    tap(frames, draw, ImGuiKey_Enter);
}

// Each round asks for what was queued while the last request was in flight.
void settle(rows_stage &staged, tests::imgui_frame &frames)
{
    for(int round = 0; round < 4; ++round)
    {
        static_cast<void>(staged.stage.shown());
        frames.draw(staged.draw());
    }
}

void check_posting(rows_stage &staged, tests::imgui_frame &frames, char letter)
{
    const double before = staged.held(letter);
    slide(frames, staged.draw(), letter, 3u);
    settle(staged, frames);
    CHECK(staged.held(letter) > before);
    CHECK(staged.published(letter).value == staged.held(letter));
}

void check_typed_entry(const char *typed, double held)
{
    CAPTURE(typed);
    rows_stage staged(two_letters);
    tests::imgui_frame frames;
    frames.draw(staged.draw());

    const std::string said = tests::reported_by(
            [&]
            {
                type_into_letter(frames, staged.draw(), 'a', typed);
                settle(staged, frames);
            });
    CHECK(said.find("finite value") == std::string::npos);
    CHECK(staged.held('a') == held);
    CHECK(staged.published('a').value == held);

    check_posting(staged, frames, 'b');
    if(held < parameter_upper)
        check_posting(staged, frames, 'a');
}

}

TEST_CASE("a value typed into a slider is held to its range, one that is no number is not asked for, and every slider posts after either", "[optimization][function][manual]")
{
    check_typed_entry("inf", parameter_upper);
    check_typed_entry("1e999", parameter_upper);
    check_typed_entry("-inf", parameter_lower);
    check_typed_entry("nan", parameter_value);
}

TEST_CASE("a value queued for a letter the function stops using is dropped, and every slider posts after it", "[optimization][function][manual]")
{
    rows_stage staged(two_letters);
    tests::imgui_frame frames;
    frames.draw(staged.draw());

    slide(frames, staged.draw(), 'b', 1u, false);
    tap(frames, staged.draw(), ImGuiKey_RightArrow);
    tap(frames, staged.draw(), ImGuiKey_Space);
    command(staged.stage.taken.inputs->scenario, [](scenario_state &state) { state.set_expression("ax^2 + c"); });
    settle(staged, frames);

    REQUIRE(staged.stage.shown()->parameters.size() == 2u);
    check_posting(staged, frames, 'a');
}

TEST_CASE("a value queued for one letter is asked for after another letter is moved", "[optimization][function][manual]")
{
    rows_stage staged(two_letters);
    tests::imgui_frame frames;
    frames.draw(staged.draw());

    slide(frames, staged.draw(), 'a', 1u, false);
    for(int step = 0; step < 4; ++step)
        tap(frames, staged.draw(), ImGuiKey_RightArrow);
    tap(frames, staged.draw(), ImGuiKey_Space);
    const double resting = staged.held('a');
    slide(frames, staged.draw(), 'b', 2u);
    const double other = staged.held('b');
    REQUIRE(resting > 1.0);
    REQUIRE(other > 1.0);

    settle(staged, frames);
    CHECK(staged.published('a').value == resting);
    CHECK(staged.published('b').value == other);
    CHECK(staged.held('a') == resting);
}
