#include "manual_stage.h"
#include "composed_curve.h"
#include "optimization_keys.h"
#include "scratch_directory.h"
#include "composed_optimization.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <filesystem>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr const char *typed_document = "<optimization><preset name=\"typed\" scenario=\"curve\"/><function source=\"manual\" expression=\"2x^3 + 5b\"/>"
                                       "<range x_lower=\"-3\" x_upper=\"3\"/><seed x=\"1\"/><solve limit=\"50\"/></optimization>";

constexpr const char *unnamed_document = "<optimization><preset name=\"unnamed\" scenario=\"curve\"/><function source=\"manual\"/><range x_lower=\"-3\" x_upper=\"3\"/>"
                                         "<seed x=\"1\"/></optimization>";

constexpr std::size_t letter_b = 'b';

config::location written(const std::string &file, const char *body)
{
    const std::filesystem::path directory = shared_scratch_directory() / "optimization_manual_save";
    std::filesystem::create_directories(directory);
    std::ofstream(directory / file, std::ios::binary | std::ios::trunc) << body << "\n";

    return config::resolve(file, directory);
}

void save(composed_optimization &composed, const config::location &at)
{
    REQUIRE(config::save(presets::optimization_keyspace(), at, composed.offered(read_optimization(at))).has_value());
}

void typed_and_left(composed_optimization &composed, tests::imgui_frame &frames, const char *typed)
{
    type_expression(frames, composed.draw("Function##curve"), typed, false, "Function##curve");
    leave_field(frames, composed.draw("Function##curve"));
    REQUIRE(composed.loop.drain().has_value());
    frames.draw_frame(composed.draw("Function##curve"));
}

void check_parameter(const typed_parameter &read, char name, double value, double lower, double upper)
{
    CHECK(read.name == name);
    CHECK(read.value == value);
    CHECK(read.range.lower == lower);
    CHECK(read.range.upper == upper);
}

void check_two_letters(const config::document &saved)
{
    CHECK(saved.text("function/source").value_or("") == "manual");
    CHECK(saved.text("function/expression").value_or("") == "2x^3 + 5b + c");
    const scenario_settings both = presets::keys::opened_settings(saved, 1u);
    REQUIRE(both.typed.parameters.size() == 2u);
    check_parameter(both.typed.parameters[0], 'b', 1.0, -5.0, 5.0);
    check_parameter(both.typed.parameters[1], 'c', 1.0, -5.0, 5.0);
}

// The minimum and the maximum share the line below the slider, entered at its leftmost.
void type_maximum(tests::imgui_frame &frames, const drawing &draw, const char *typed)
{
    stand_on(frames, draw, "Function##curve", letter_b, "b");
    tap(frames, draw, ImGuiKey_DownArrow);
    for(ImGuiID standing = 0; standing != standing_on();)
    {
        standing = standing_on();
        tap(frames, draw, ImGuiKey_LeftArrow);
    }
    tap(frames, draw, ImGuiKey_RightArrow);
    REQUIRE(standing_on() == control_id("Function##curve", letter_b, "max"));

    tap(frames, draw, ImGuiKey_Space);
    for(const char *at = typed; *at != 0; ++at)
        ImGui::GetIO().AddInputCharacter(static_cast<unsigned int>(*at));
    frames.draw_frame(draw);
    tap(frames, draw, ImGuiKey_Enter);
}

}

TEST_CASE("the Function window saves the text it last asked for with an instance per parameter, and takes out a letter the text no longer uses",
          "[presets][optimization][documents][typed]")
{
    const config::location at = written("letters.xml", typed_document);
    {
        composed_optimization composed(at, {}, "typed");
        tests::imgui_frame frames;
        typed_and_left(composed, frames, "2x^3 + 5b + c");
        save(composed, at);

        check_two_letters(read_optimization(at));

        typed_and_left(composed, frames, "2x^3 + 5b");
        save(composed, at);
        CHECK(composed.offered(read_optimization(at)).empty());
    }

    const config::document reopened = read_optimization(at);
    CHECK(reopened.identities("parameters/parameter") == std::vector<std::string>{"b"});
    const composed_optimization second(at, {}, "typed");
    CHECK(second.offered(reopened).empty());
}

TEST_CASE("moving a parameter's slider and its maximum moves the value and the upper bound saved", "[presets][optimization][documents][typed]")
{
    const config::location at = written("moved.xml", typed_document);
    composed_optimization composed(at, {}, "typed");
    const drawing function = composed.draw("Function##curve");
    tests::imgui_frame frames;
    frames.draw(function);

    type_maximum(frames, function, "8");
    REQUIRE(composed.loop.drain().has_value());
    stand_on(frames, function, "Function##curve", letter_b, "b");
    tap(frames, function, ImGuiKey_Space);
    tap(frames, function, ImGuiKey_RightArrow);
    tap(frames, function, ImGuiKey_Space);
    REQUIRE(composed.loop.drain().has_value());
    frames.draw_frame(function);
    save(composed, at);

    const scenario_settings saved = presets::keys::opened_settings(read_optimization(at), 1u);
    REQUIRE(saved.typed.parameters.size() == 1u);
    CHECK(saved.typed.parameters.front().value > 1.0);
    CHECK(saved.typed.parameters.front().range.lower == -5.0);
    CHECK(saved.typed.parameters.front().range.upper == 8.0);
}

TEST_CASE("a composition registering no function composes, draws and saves a typed one", "[presets][optimization][documents][typed]")
{
    const config::location at = written("unnamed.xml", unnamed_document);
    {
        composed_optimization composed(at, {}, "unnamed");
        tests::imgui_frame frames;
        take_entry_on(frames, composed.draw("Function##curve"), "Function##curve", "Source", 0u);
        REQUIRE(composed.loop.drain().has_value());
        typed_and_left(composed, frames, "ax^2 - 1");
        save(composed, at);
    }

    const config::document reopened = read_optimization(at);
    CHECK(reopened.text("function/name").value_or("").empty());
    const scenario_settings saved = presets::keys::opened_settings(reopened, 1u);
    CHECK(saved.typed.source == function_source::manual);
    CHECK(saved.typed.expression == "ax^2 - 1");
    REQUIRE(saved.typed.parameters.size() == 1u);
    check_parameter(saved.typed.parameters.front(), 'a', 1.0, -5.0, 5.0);
}
