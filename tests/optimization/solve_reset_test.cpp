#include "drawn_graph.h"
#include "drawn_surface.h"
#include "composed_curve.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/solve_window.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/plot_window.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr std::size_t seed_row = 0;
constexpr std::size_t run_row  = 2;

double bowl_value(double x)
{
    return x * x - 1.0;
}

double bowl_slope(double x)
{
    return 2.0 * x;
}

double circle_value(double x, double y)
{
    return x * x + y * y - 1.0;
}

Eigen::Vector2d circle_gradient(double x, double y)
{
    return Eigen::Vector2d(2.0 * x, 2.0 * y);
}

// The curve's graph and its Solve window over one curve.
struct reset_stage
{
    std::shared_ptr<solve_window> panel;
    graph_stage stage;

    reset_stage(const registered_curve &curve, std::uint32_t limit)
            : panel()
            , stage(curve, composer(panel), false)
    {
        command(stage.taken.inputs->scenario, [limit](scenario_state &state) { state.run(Eigen::VectorXd::Constant(1, 3.0), limit); });
        static_cast<void>(stage.drained());
    }

    static scenario_window_composer composer(std::shared_ptr<solve_window> &kept)
    {
        return [&kept](const scenario_window_inputs &offered)
        {
            kept = std::make_shared<solve_window>("Solve", offered);

            return std::vector<std::shared_ptr<scene::imgui_window>>{curve_graph(offered, "Graph"), kept};
        };
    }

    drawing draw() const
    {
        return [shown = panel] { shown->render(); };
    }

    std::shared_ptr<const scenario_snapshot> pressed(tests::imgui_frame &frames, const char *label)
    {
        press_along(frames, draw(), run_row, "Solve", label);
        static_cast<void>(stage.drained());

        return stage.taken.inputs->seen.read();
    }
};

}

TEST_CASE("reset in the solve window returns the run to its seed, and step once takes the first step again", "[optimization][solve]")
{
    reset_stage staged_panel(cubic(), 5u);
    REQUIRE(staged_panel.stage.taken.inputs->seen.read()->run.iterates.size() == 6u);

    tests::imgui_frame frames;
    const std::shared_ptr<const scenario_snapshot> reset = staged_panel.pressed(frames, "Reset");
    CHECK(reset->run.iterates == std::vector<Eigen::VectorXd>{Eigen::VectorXd::Constant(1, 3.0)});
    CHECK(reset->step == 0u);
    CHECK(run_status(*reset) == "Steps taken: 0.");

    const std::shared_ptr<const scenario_snapshot> stepped = staged_panel.pressed(frames, "Step once");
    REQUIRE(stepped->run.iterates.size() == 2u);
    CHECK(stepped->run.iterates[0][0] == 3.0);
    CHECK(stepped->run.iterates[1][0] == 3.0 - cubic_value(3.0) / cubic_slope(3.0));
    CHECK(stepped->step == 0u);
}

TEST_CASE("reset solves from the seed in the window's field, typed and not yet run", "[optimization][solve]")
{
    reset_stage staged_panel(registered_curve("bowl", &bowl_value, &bowl_slope), 70u);
    tests::imgui_frame frames;
    type_into(frames, staged_panel.draw(), seed_row, "1");

    const std::shared_ptr<const scenario_snapshot> reset = staged_panel.pressed(frames, "Reset");
    CHECK(reset->run.iterates == std::vector<Eigen::VectorXd>{Eigen::VectorXd::Constant(1, 1.0)});
    CHECK(run_status(*reset) == "Converged. Steps taken: 0.");
}

TEST_CASE("a run holding its seed alone draws the seed's marker and no step on the curve", "[optimization][solve][graph]")
{
    reset_stage staged_panel(cubic(), 70u);
    tests::imgui_frame frames;
    static_cast<void>(staged_panel.pressed(frames, "Reset"));

    const scene::plot_frame frame      = drawn_now(staged_panel.stage.taken);
    const scene::plot_series *iterates = named(frame, "Iterates");
    REQUIRE(iterates != nullptr);
    CHECK(iterates->abscissa == std::vector<double>{3.0});
    for(const char *name : {"tangent at x_k", "zero of the tangent", "x_(k+1)"})
        CHECK(named(frame, name) == nullptr);
}

TEST_CASE("a run holding its seed alone draws one iterate marker and no step on the surface", "[optimization][solve][surface]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const opened taken = open_surface_with(built.site, {registered_surface("circle", &circle_value, &circle_gradient)}, baseline(),
                                           scenario_settings{"circle", {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, Eigen::Vector2d(1.5, 0.5), 70u, 0u, {}});
    REQUIRE(settled(loop, taken)->run.iterates.size() > 1u);

    command(taken.inputs->scenario, [](scenario_state &state) { state.reset(Eigen::Vector2d(1.5, 0.5)); });
    REQUIRE(loop.drain().has_value());
    taken.composed->stencil->render();

    threepp::Scene &scene = *built.scene;
    CHECK(shown_markers(scene).size() == 1u);
    for(const char *name : {"tangent plane", "zero line", "nearest point"})
        CHECK_FALSE(drawn_visible(scene, name));
    taken.composed->tear_down();
}
