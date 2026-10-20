#include "drawn_graph.h"
#include "composed_curve.h"

#include "panel_keys.h"
#include "imgui_frame.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/solve_window.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/optimization/projected/solver.h"

#include "praxis/scene/plot_window.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <utility>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;

namespace {

double bowl_value(double x)
{
    return x * x - 1.0;
}

double bowl_slope(double x)
{
    return 2.0 * x;
}

registered_curve boxed_cubic()
{
    return registered_curve("boxed", &cubic_value, &cubic_slope, box{Eigen::VectorXd::Constant(1, 0.0), Eigen::VectorXd::Constant(1, 2.5)});
}

scenario_settings seeded(std::string selected, double seed)
{
    return scenario_settings{std::move(selected), interval{-4.0, 4.0}, seed, 70u, 0u};
}

bool near(double value, double expected)
{
    return std::abs(value - expected) <= 1.0e-12 * std::max(1.0, std::abs(expected));
}

void check_next_iterate(const scene::plot_frame &frame, double next)
{
    const scene::plot_series *recorded = named(frame, "x_(k+1)");
    REQUIRE(recorded != nullptr);
    CHECK(recorded->mark == scene::plot_mark::segments);
    CHECK(recorded->abscissa == std::vector<double>{next, next});
    CHECK(recorded->ordinate == std::vector<double>{0.0, cubic_value(next)});
}

capabilities projecting()
{
    return capabilities{solver_ops{&projected::solve}};
}

scenario_window_composer graph_and_solve(std::shared_ptr<scene::imgui_window> &graph, std::shared_ptr<solve_window> &solving)
{
    return [&graph, &solving](const scenario_window_inputs &offered)
    {
        graph   = curve_graph(offered, "Graph");
        solving = std::make_shared<solve_window>("Solve", offered);

        return std::vector<std::shared_ptr<scene::imgui_window>>{graph, solving};
    };
}

}

TEST_CASE("the selected step draws the tangent at x_k, its zero and the recorded next iterate", "[optimization][graph]")
{
    graph_stage stage(cubic(), baseline(), opening());
    REQUIRE(stage.ran->run.iterates.size() >= 3u);

    const scene::plot_frame frame = stage.at_step(1u);
    const double x                = stage.ran->run.iterates[1][0];
    const double next             = stage.ran->run.iterates[2][0];

    const scene::plot_series *tangent = named(frame, "tangent at x_k");
    REQUIRE(tangent != nullptr);
    check_on_line(*tangent, x, cubic_value(x), cubic_slope(x));
    const interval span = drawn_span(frame, 2u);
    for(const double y : tangent->ordinate)
        CHECK((y >= span.lower - 1.0e-9 && y <= span.upper + 1.0e-9));

    const scene::plot_series *zero = named(frame, "zero of the tangent");
    REQUIRE(zero != nullptr);
    CHECK(zero->mark == scene::plot_mark::markers);
    REQUIRE(zero->abscissa.size() == 1u);
    CHECK(near(zero->abscissa[0], next));
    CHECK(zero->ordinate == std::vector<double>{0.0});

    check_next_iterate(frame, next);
    CHECK(named(frame, "bounds") == nullptr);
}

TEST_CASE("under a projected step the tangent's zero and the recorded next iterate are drawn apart", "[optimization][graph]")
{
    graph_stage stage(boxed_cubic(), projecting(), seeded("boxed", 1.5));
    REQUIRE(stage.ran->run.iterates.size() >= 2u);

    const scene::plot_frame frame = stage.at_step(0u);
    const double crossing         = 1.5 - cubic_value(1.5) / cubic_slope(1.5);
    const double next             = stage.ran->run.iterates[1][0];
    REQUIRE(std::abs(crossing - next) > 1.0e-6);

    const scene::plot_series *zero = named(frame, "zero of the tangent");
    REQUIRE(zero != nullptr);
    REQUIRE(zero->abscissa.size() == 1u);
    CHECK(near(zero->abscissa[0], crossing));
    check_next_iterate(frame, next);
}

TEST_CASE("a horizontal tangent is drawn with no zero", "[optimization][graph]")
{
    graph_stage stage(registered_curve("bowl", &bowl_value, &bowl_slope), baseline(), seeded("bowl", 0.0));

    const scene::plot_frame frame     = stage.at_step(0u);
    const scene::plot_series *tangent = named(frame, "tangent at x_k");
    REQUIRE(tangent != nullptr);
    check_on_line(*tangent, 0.0, -1.0, 0.0);
    CHECK(tangent->abscissa == std::vector<double>{-4.0, 4.0});
    CHECK(named(frame, "zero of the tangent") == nullptr);
}

TEST_CASE("a box draws its two ends as vertical segments across the drawn ordinates", "[optimization][graph]")
{
    graph_stage stage(boxed_cubic(), projecting(), seeded("boxed", 1.5));

    const scene::plot_frame frame   = stage.at_step(0u);
    const interval span             = drawn_span(frame, 2u);
    const scene::plot_series *edges = named(frame, "bounds");
    REQUIRE(edges != nullptr);
    CHECK(edges->mark == scene::plot_mark::segments);
    CHECK(edges->abscissa == std::vector<double>{0.0, 0.0, 2.5, 2.5});
    CHECK(edges->ordinate == std::vector<double>{span.lower, span.upper, span.lower, span.upper});
}

TEST_CASE("an iterate beyond the range widens neither the box's ends nor the ordinates they span", "[optimization][graph]")
{
    graph_stage stage(boxed_cubic(), baseline(), scenario_settings{"boxed", interval{-3.0, 3.0}, -2.25, 200u, 0u});
    REQUIRE(std::ranges::any_of(stage.ran->run.iterates, [](const Eigen::VectorXd &x) { return std::abs(x[0]) > 10.0; }));

    const scene::plot_frame frame   = stage.at_step(0u);
    const interval span             = drawn_span(frame, 2u);
    const scene::plot_series *edges = named(frame, "bounds");
    REQUIRE(edges != nullptr);
    CHECK(edges->ordinate == std::vector<double>{span.lower, span.upper, span.lower, span.upper});
    CHECK(span.upper == cubic_value(3.0));
}

TEST_CASE("a Shift press over the curve seeds a run at the pressed abscissa, which the solve window then runs from", "[optimization][graph]")
{
    std::shared_ptr<scene::imgui_window> graph;
    std::shared_ptr<solve_window> solving;
    graph_stage stage(cubic(), graph_and_solve(graph, solving), false);

    const pressed_at pressed = shift_press(graph);
    REQUIRE(stage.loop.drain().has_value());
    const std::shared_ptr<const scenario_snapshot> picked = stage.taken.inputs->seen.read();
    const double seed                                     = picked->run.seed[0];
    CHECK(std::abs(seed - pressed.abscissa) <= pressed.pixel);
    CHECK(picked->run.iterates.front()[0] == seed);

    tests::imgui_frame panel;
    const drawing draw_solve = [&solving] { solving->render(); };
    stand_below_top(panel, draw_solve, 2u);
    tap(panel, draw_solve, ImGuiKey_LeftArrow);
    tap(panel, draw_solve, ImGuiKey_Space);
    REQUIRE(stage.loop.drain().has_value());
    CHECK(stage.taken.inputs->seen.read() != picked);
    CHECK(stage.taken.inputs->seen.read()->run.seed[0] == seed);
}
