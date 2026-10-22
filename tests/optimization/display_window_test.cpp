#include "drawn_graph.h"
#include "composed_curve.h"

#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/display_window.h"
#include "praxis/optimization/compose_scenario.h"

#include "praxis/scene/plot_window.h"
#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <memory>
#include <vector>

using namespace praxis;
using namespace praxis::fixture;

namespace {

registered_curve boxed()
{
    return registered_curve("cubic", &cubic_value, &cubic_slope, box{Eigen::VectorXd::Constant(1, 0.0), Eigen::VectorXd::Constant(1, 3.5)});
}

// The graph and the Display window over one curve, the window kept for the case to press.
struct displayed_stage
{
    std::shared_ptr<display_window> panel;
    graph_stage stage;

    explicit displayed_stage(const registered_curve &curve)
            : panel()
            , stage(curve, composer(panel), false)
    {
    }

    static scenario_window_composer composer(std::shared_ptr<display_window> &kept)
    {
        return [&kept](const scenario_window_inputs &offered)
        {
            kept = std::make_shared<display_window>("Display", offered);

            return std::vector<std::shared_ptr<scene::imgui_window>>{curve_graph(offered, "Graph"), kept};
        };
    }

    scene::plot_frame pressed(const char *label)
    {
        tests::imgui_frame frames;
        press_on(frames, [shown = panel] { shown->render(); }, "Display", label);

        return stage.drained();
    }
};

}

TEST_CASE("a curve's Display window hides the iterates the graph draws", "[optimization][graph][view]")
{
    displayed_stage displayed(cubic());
    REQUIRE(named(displayed.stage.drained(), "Iterates") != nullptr);

    const scene::plot_frame hidden = displayed.pressed("Iterates");
    CHECK(named(hidden, "Iterates") == nullptr);
    CHECK(named(hidden, "g(x)") != nullptr);

    CHECK(named(displayed.pressed("Iterates"), "Iterates") != nullptr);
}

TEST_CASE("a curve's Display window hides the selected step's construction and the box", "[optimization][graph][view]")
{
    displayed_stage displayed(boxed());
    const scene::plot_frame shown = displayed.stage.at_step(0u);
    for(const char *name : {"tangent at x_k", "zero of the tangent", "x_(k+1)", "bounds"})
        REQUIRE(named(shown, name) != nullptr);

    const scene::plot_frame stepless = displayed.pressed("Step");
    for(const char *name : {"tangent at x_k", "zero of the tangent", "x_(k+1)"})
        CHECK(named(stepless, name) == nullptr);
    CHECK(named(stepless, "bounds") != nullptr);
    CHECK(named(stepless, "Iterates") != nullptr);

    const scene::plot_frame boxless = displayed.pressed("Box");
    CHECK(named(boxless, "bounds") == nullptr);
    CHECK(named(boxless, "g(x)") != nullptr);
}

TEST_CASE("a curve composed without a Display window draws every part", "[optimization][graph][view]")
{
    graph_stage stage(boxed(), baseline(), opening());
    const scene::plot_frame shown = stage.at_step(0u);
    for(const char *name : {"g(x)", "Iterates", "tangent at x_k", "zero of the tangent", "x_(k+1)", "bounds"})
        CHECK(named(shown, name) != nullptr);
}
