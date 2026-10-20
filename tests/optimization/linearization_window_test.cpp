#include "drawn_graph.h"
#include "composed_curve.h"

#include "panel_keys.h"
#include "imgui_frame.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/scenario_snapshot.h"
#include "praxis/optimization/linearization_window.h"

#include "praxis/scene/plot_window.h"
#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr std::size_t point_row  = 0;
constexpr std::size_t slider_row = 1;

registered_curve wrong_by_sign()
{
    return registered_curve("cubic", &cubic_value, [](double x) { return -cubic_slope(x); });
}

// The graph and the Linearization window over one curve, the window kept for the case to draw.
struct linearizing_stage
{
    std::shared_ptr<linearization_window> panel;
    graph_stage stage;

    linearizing_stage(const registered_curve &curve, bool checks_derivative)
            : panel()
            , stage(curve, composer(panel), checks_derivative)
    {
    }

    static scenario_window_composer composer(std::shared_ptr<linearization_window> &kept)
    {
        return [&kept](const scenario_window_inputs &offered)
        {
            kept = std::make_shared<linearization_window>("Linearization", offered);

            return std::vector<std::shared_ptr<scene::imgui_window>>{curve_graph(offered, "Graph"), kept};
        };
    }

    drawing draw() const
    {
        return [shown = panel] { shown->render(); };
    }

    scenario_linearization at() const
    {
        const std::shared_ptr<const scenario_snapshot> shown = stage.taken.inputs->seen.read();
        REQUIRE(shown->linearization.has_value());

        return *shown->linearization;
    }

    std::vector<std::string> lines(bool checks_derivative) const
    {
        return linearization_lines(at(), checks_derivative);
    }
};

// The point typed into the field, after the work strand has evaluated it.
scene::plot_frame typed_point(linearizing_stage &staged_panel, const char *typed)
{
    tests::imgui_frame frames;
    type_into(frames, staged_panel.draw(), point_row, typed);

    return staged_panel.stage.drained();
}

int drawn_vertices(const linearizing_stage &staged_panel)
{
    tests::imgui_frame frames;
    frames.draw(staged_panel.draw());

    return frames.vertices();
}

}

TEST_CASE("a point typed into the field is evaluated, shown with its value and derivative, and its tangent drawn", "[optimization][linearization]")
{
    linearizing_stage staged_panel(cubic(), false);
    CHECK(staged_panel.at().point[0] == 0.0);

    const scene::plot_frame frame   = typed_point(staged_panel, "1.5");
    const scenario_linearization at = staged_panel.at();
    REQUIRE(at.point.size() == 1);
    CHECK(at.point[0] == 1.5);
    REQUIRE(at.error.has_value());
    REQUIRE(at.jacobian.has_value());
    CHECK(-(*at.error)[0] == cubic_value(1.5));
    CHECK((*at.jacobian)(0, 0) == cubic_slope(1.5));

    const std::vector<std::string> lines = staged_panel.lines(false);
    REQUIRE(lines.size() == 2u);
    CHECK_THAT(lines[0], Catch::Matchers::ContainsSubstring(std::format("{:.10g}", cubic_value(1.5))));
    CHECK_THAT(lines[1], Catch::Matchers::ContainsSubstring(std::format("{:.10g}", cubic_slope(1.5))));

    const scene::plot_series *tangent = named(frame, "tangent at p");
    REQUIRE(tangent != nullptr);
    check_on_line(*tangent, 1.5, cubic_value(1.5), cubic_slope(1.5));
}

TEST_CASE("the slider moves the point along the range and the point is evaluated there", "[optimization][linearization]")
{
    linearizing_stage staged_panel(cubic(), false);
    static_cast<void>(typed_point(staged_panel, "1.5"));

    tests::imgui_frame frames;
    tweak_from(frames, staged_panel.draw(), slider_row, 3u);
    const scene::plot_frame frame   = staged_panel.stage.drained();
    const scenario_linearization at = staged_panel.at();
    CHECK(at.point[0] != 1.5);
    CHECK((at.point[0] >= -4.0 && at.point[0] <= 4.0));
    CHECK(-(*at.error)[0] == cubic_value(at.point[0]));

    const scene::plot_series *tangent = named(frame, "tangent at p");
    REQUIRE(tangent != nullptr);
    check_on_line(*tangent, at.point[0], cubic_value(at.point[0]), cubic_slope(at.point[0]));
}

TEST_CASE("with the check opted in, a derivative wrong by its sign is stated and both tangents are drawn", "[optimization][linearization]")
{
    linearizing_stage staged_panel(wrong_by_sign(), true);
    const scene::plot_frame frame = typed_point(staged_panel, "1.5");

    const scenario_linearization at = staged_panel.at();
    REQUIRE(at.check.has_value());
    CHECK(at.check->disagrees);
    CHECK(std::abs(at.check->central(0, 0) - cubic_slope(1.5)) <= 1.0e-6);

    const std::vector<std::string> lines = staged_panel.lines(true);
    REQUIRE(lines.size() == 4u);
    CHECK_THAT(lines[3], Catch::Matchers::ContainsSubstring("The derivative handed in and the central difference differ at x = 1.5"));

    const scene::plot_series *estimated = named(frame, "central difference at p");
    REQUIRE(estimated != nullptr);
    check_on_line(*estimated, 1.5, cubic_value(1.5), at.check->central(0, 0));
    REQUIRE(named(frame, "tangent at p") != nullptr);
}

TEST_CASE("with the check opted in, a correct derivative raises nothing and only the central difference is shown", "[optimization][linearization]")
{
    linearizing_stage staged_panel(cubic(), true);
    const scene::plot_frame frame = typed_point(staged_panel, "1.5");

    const scenario_linearization at = staged_panel.at();
    REQUIRE(at.check.has_value());
    CHECK_FALSE(at.check->disagrees);
    CHECK(staged_panel.lines(true).size() == 3u);
    CHECK(named(frame, "central difference at p") == nullptr);
    CHECK(named(frame, "tangent at p") != nullptr);
}

TEST_CASE("with the check not opted in, nothing of it is evaluated, written or drawn", "[optimization][linearization]")
{
    linearizing_stage checked(cubic(), true);
    static_cast<void>(typed_point(checked, "1.5"));
    linearizing_stage unchecked(wrong_by_sign(), false);
    const scene::plot_frame frame = typed_point(unchecked, "1.5");

    CHECK_FALSE(unchecked.at().check.has_value());
    CHECK(named(frame, "central difference at p") == nullptr);

    linearizing_stage unchecked_correct(cubic(), false);
    static_cast<void>(typed_point(unchecked_correct, "1.5"));
    std::vector<std::string> less_the_check = checked.lines(true);
    less_the_check.pop_back();
    CHECK(unchecked_correct.lines(false) == less_the_check);
    CHECK(drawn_vertices(unchecked_correct) < drawn_vertices(checked));
}

TEST_CASE("a composition leaving the Linearization window out composes, runs and draws no tangent at a point", "[optimization][linearization]")
{
    graph_stage stage(cubic(), baseline(), opening());

    REQUIRE(stage.ran != nullptr);
    CHECK_FALSE(stage.ran->linearization.has_value());
    CHECK_FALSE(stage.ran->run.refused.has_value());
    CHECK(named(drawn_now(stage.taken), "tangent at p") == nullptr);
}
