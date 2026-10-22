#include "drawn_graph.h"
#include "composed_curve.h"
#include "minimized_stage.h"

#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/display_window.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"

#include "praxis/scene/plot_window.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <vector>
#include <utility>

using namespace praxis;
using namespace praxis::fixture;

namespace {

// The graph and the Display window over the quartic minimized from 2, step 0 selected.
struct displayed_minimum
{
    scheduler::scheduler loop;
    staged built;
    std::shared_ptr<display_window> panel;
    opened taken;

    explicit displayed_minimum(minimizer_slot method)
            : loop(scheduler::inline_workers)
            , built(loop)
            , panel()
            , taken(open_composing(built.site, {quartic_curve()}, baseline(), minimum_from(quartic_name, 2.0, method, 0.05, 50u), composer(panel)))
    {
        REQUIRE(settled(loop, taken) != nullptr);
        command(taken.inputs->scenario, [](scenario_state &state) { state.select_step(0u); });
    }

    ~displayed_minimum()
    {
        taken.composed->tear_down();
    }

    static scenario_window_composer composer(std::shared_ptr<display_window> &kept)
    {
        return [&kept](const scenario_window_inputs &offered)
        {
            kept = std::make_shared<display_window>("Display", offered);

            return std::vector<std::shared_ptr<scene::imgui_window>>{curve_graph(offered, "Graph"), kept};
        };
    }

    scene::plot_frame drawn()
    {
        REQUIRE(loop.drain().has_value());

        return drawn_now(taken);
    }

    scene::plot_frame pressed(const char *label)
    {
        tests::imgui_frame frames;
        press_on(frames, [shown = panel] { shown->render(); }, "Display", label);

        return drawn();
    }
};

}

TEST_CASE("the Display window's Step switch hides and shows the model, its vertex and the next iterate", "[optimization][graph][objective][view]")
{
    const std::vector<std::pair<minimizer_slot, std::vector<const char *>>> drawn_by{{minimizer_slot::newton, {"quadratic model at x_k", "vertex", "x_(k+1)"}},
                                                                                     {minimizer_slot::gradient_descent, {"tangent at x_k", "x_(k+1)"}}};
    for(const auto &[method, names] : drawn_by)
    {
        displayed_minimum displayed(method);
        const scene::plot_frame shown    = displayed.drawn();
        const scene::plot_frame stepless = displayed.pressed("Step");
        const scene::plot_frame restored = displayed.pressed("Step");
        for(const char *name : names)
        {
            CHECK(named(shown, name) != nullptr);
            CHECK(named(stepless, name) == nullptr);
            CHECK(named(restored, name) != nullptr);
        }
        CHECK(named(stepless, "f(x)") != nullptr);
        CHECK(named(stepless, "Iterates") != nullptr);
    }
}
