#include "typed_stage.h"
#include "manual_stage.h"
#include "composed_curve.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/scenario_view.h"
#include "praxis/optimization/scenario_snapshot.h"
#include "praxis/optimization/linearization_window.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <imgui.h>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;

namespace {

double typed_cubic(double x)
{
    return 2.0 * x * x * x + 5.0;
}

void check_typed_cubic_drawn(manual_stage &stage)
{
    const scene::plot_reading read = curve_reading(stage.taken.inputs->seen, scenario_view());
    REQUIRE(read.frames.size() == 1u);
    const scene::plot_series &curve = read.frames.front().series.front();
    REQUIRE(curve.abscissa.size() > 1u);
    for(std::size_t at = 0; at < curve.abscissa.size(); ++at)
        CHECK(within(curve.ordinate[at], typed_cubic(curve.abscissa[at])));
}

bool nothing_drawn(manual_stage &stage)
{
    const scene::plot_reading read = curve_reading(stage.taken.inputs->seen, scenario_view());

    return read.frames.empty() || read.frames.front().series.empty() || read.frames.front().series.front().abscissa.empty();
}

// Every control the keyboard reaches walking the panel down from its first.
std::vector<ImGuiID> walked(tests::imgui_frame &frames, const drawing &draw)
{
    reach_top(frames, draw);
    std::vector<ImGuiID> reached;
    for(ImGuiID standing = 0; standing != standing_on();)
    {
        standing = standing_on();
        reached.push_back(standing);
        tap(frames, draw, ImGuiKey_DownArrow);
    }

    return reached;
}

}

TEST_CASE("text typed into the Manual field is echoed, asked for once it parses and drawn", "[optimization][function][manual]")
{
    manual_stage stage(opening());
    tests::imgui_frame frames;
    select_manual(frames, stage.draw());

    type_expression(frames, stage.draw(), "2x^3 + 5b");
    CHECK(stage.written(frames).find("2*x^3 + 5*b") != std::string::npos);
    leave_field(frames, stage.draw());
    const std::shared_ptr<const scenario_snapshot> drawn = stage.drained();
    REQUIRE(drawn != nullptr);
    check_typed_cubic_drawn(stage);
    stand_on(frames, stage.draw(), "Function", static_cast<std::size_t>('b'), "b");

    type_expression(frames, stage.draw(), " +", true);
    const std::string refused = stage.written(frames);
    CHECK(refused.find("At character 12: expected an operand") != std::string::npos);
    leave_field(frames, stage.draw());
    CHECK(stage.drained() == drawn);
}

TEST_CASE("a composition registering no function opens a typed one, offers both sources and draws what is typed", "[optimization][function][manual]")
{
    manual_stage stage(typed_curve("x - 1", 1.0), {});
    tests::imgui_frame frames;
    const std::shared_ptr<const scenario_snapshot> opened = stage.drained();
    REQUIRE(opened != nullptr);
    CHECK_FALSE(nothing_drawn(stage));

    take_entry_on(frames, stage.draw(), "Function", "Source", 0u);
    CHECK(stage.drained() == opened);
    CHECK(stage.written(frames).find("f(x) =") != std::string::npos);

    type_expression(frames, stage.draw(), "2x^3 + 5b");
    leave_field(frames, stage.draw());
    static_cast<void>(stage.drained());
    check_typed_cubic_drawn(stage);
}

TEST_CASE("an opened text longer than the field stands refused at the field's end, and leaving it unedited asks for nothing", "[optimization][function][manual]")
{
    manual_stage stage(typed_curve("x" + std::string(600, ' ') + "+ 1", 1.0));
    tests::imgui_frame frames;
    static_cast<void>(stage.drained());

    CHECK(stage.written(frames).find("expected the end within") != std::string::npos);
    stand_on(frames, stage.draw(), "Function", "##expression");
    tap(frames, stage.draw(), ImGuiKey_Space);
    leave_field(frames, stage.draw());
    static_cast<void>(stage.drained());
    CHECK(nothing_drawn(stage));
}

TEST_CASE("an opened text that does not parse stands in the field with its error and nothing drawn until a text that parses is left", "[optimization][function][manual]")
{
    manual_stage stage(typed_curve("2x^3 +", 1.0));
    tests::imgui_frame frames;
    static_cast<void>(stage.drained());

    const std::string refused = stage.written(frames);
    CHECK(refused.find("2x^3 +>>") != std::string::npos);
    CHECK(refused.find("At character 7: expected an operand") != std::string::npos);
    CHECK(nothing_drawn(stage));

    type_expression(frames, stage.draw(), "2x^3 + 5b");
    leave_field(frames, stage.draw());
    static_cast<void>(stage.drained());
    check_typed_cubic_drawn(stage);
}

TEST_CASE("a surface's field is labeled by both variables, and only its parameters get a row", "[optimization][function][manual]")
{
    manual_stage stage(typed_over("x^2 + ay^2", {interval{-3.0, 3.0}, interval{-3.0, 3.0}}, Eigen::Vector2d(1.0, 1.0)));
    tests::imgui_frame frames;
    static_cast<void>(stage.drained());
    CHECK(stage.written(frames).find("f(x, y) =") != std::string::npos);

    const std::vector<ImGuiID> reached = walked(frames, stage.draw());
    CHECK(std::ranges::find(reached, control_id("Function", static_cast<std::size_t>('a'), "a")) != reached.end());
    CHECK(std::ranges::find(reached, control_id("Function", static_cast<std::size_t>('y'), "y")) == reached.end());
    CHECK(std::ranges::find(reached, control_id("Function", static_cast<std::size_t>('x'), "x")) == reached.end());
}

TEST_CASE("a typed function's linearization writes no derivative check even where the composition opted in", "[optimization][function][manual]")
{
    typed_stage stage(typed_curve("2x^3 + 5b", 1.0), {cubic()}, true);
    linearization_window shown("Linearization", *stage.taken.inputs);
    const drawing draw = [&shown] { shown.render(); };
    tests::imgui_frame frames;
    type_into(frames, draw, 0u, "1.5");

    const std::shared_ptr<const scenario_snapshot> at = stage.shown();
    REQUIRE(at->linearization.has_value());
    CHECK_FALSE(at->linearization->check.has_value());

    std::string written;
    frames.draw_frame(
            [&shown, &written]
            {
                ImGui::LogToBuffer();
                shown.render();
                written = GImGui->LogBuffer.c_str();
                ImGui::LogFinish();
            });
    CHECK(written.find("g'(x) = ") != std::string::npos);
    CHECK(written.find("central difference") == std::string::npos);
}
