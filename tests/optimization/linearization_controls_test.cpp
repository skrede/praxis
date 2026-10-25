#include "drawn_graph.h"
#include "drawn_surface.h"
#include "composed_curve.h"

#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"
#include "praxis/optimization/linearization_window.h"

#include "praxis/scene/plot_window.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr const char *named_at_p[] = {"linearization point p", "tangent at p", "central difference at p"};

registered_curve wrong_by_sign()
{
    return registered_curve("cubic", &cubic_value, [](double x) { return -cubic_slope(x); });
}

double circle_value(double x, double y)
{
    return x * x + y * y - 1.0;
}

Eigen::Vector2d swapped_gradient(double x, double y)
{
    return Eigen::Vector2d(2.0 * y, 2.0 * x);
}

void press_show(const std::shared_ptr<scene::imgui_window> &panel)
{
    tests::imgui_frame frames;
    press_on(frames, [panel] { panel->render(); }, "Linearization", "Show");
}

scenario_window_composer graph_and_linearization(std::shared_ptr<scene::imgui_window> &kept)
{
    return [&kept](const scenario_window_inputs &offered)
    {
        kept = std::make_shared<linearization_window>("Linearization", offered);

        return std::vector<std::shared_ptr<scene::imgui_window>>{curve_graph(offered, "Graph"), kept};
    };
}

// A surface whose gradient disagrees with its central difference, linearized off its zero curve.
struct linearized_surface
{
    scheduler::scheduler loop;
    staged built;
    std::shared_ptr<scene::imgui_window> panel;
    opened taken;

    explicit linearized_surface(std::vector<interval> ranges = {interval{-2.0, 2.0}, interval{-2.0, 2.0}})
            : loop(scheduler::inline_workers)
            , built(loop)
            , panel()
            , taken(open_surface_with(
                      built.site, {registered_surface("circle", &circle_value, &swapped_gradient)}, baseline(),
                      scenario_settings{"circle", std::move(ranges), Eigen::Vector2d(1.5, 0.5), 70u, 0u, {}, {}},
                      [this](const scenario_window_inputs &offered)
                      {
                          panel = std::make_shared<linearization_window>("Linearization", offered);
                          return std::vector<std::shared_ptr<scene::imgui_window>>{panel};
                      },
                      true))
    {
        static_cast<void>(settled(loop, taken));
        command(taken.inputs->scenario, [](scenario_state &state) { state.linearize(Eigen::Vector2d(1.5, 0.5)); });
        REQUIRE(loop.drain().has_value());
        taken.composed->stencil->render();
    }

    ~linearized_surface()
    {
        taken.composed->tear_down();
    }

    void show()
    {
        press_show(panel);
        taken.composed->stencil->render();
    }
};

// The point the latest publication carries once the window's Reset is pressed.
Eigen::VectorXd reset_point(const std::shared_ptr<scene::imgui_window> &panel, scheduler::scheduler &loop, const opened &taken)
{
    tests::imgui_frame frames;
    press_on(frames, [panel] { panel->render(); }, "Linearization", "Reset");
    REQUIRE(loop.drain().has_value());
    const std::shared_ptr<const scenario_snapshot> shown = taken.inputs->seen.read();
    REQUIRE(shown->linearization.has_value());

    return shown->linearization->point;
}

}

TEST_CASE("the Linearization window's Show switch starts off and shows then hides the point and its tangents on a curve", "[optimization][graph][linearization]")
{
    std::shared_ptr<scene::imgui_window> panel;
    graph_stage stage(wrong_by_sign(), graph_and_linearization(panel), true);
    const scene::plot_frame opening = stage.drained();
    for(const char *name : named_at_p)
        CHECK(named(opening, name) == nullptr);
    CHECK(named(opening, "Iterates") != nullptr);

    press_show(panel);
    const scene::plot_frame shown = stage.drained();
    for(const char *name : named_at_p)
        CHECK(named(shown, name) != nullptr);

    press_show(panel);
    const scene::plot_frame hidden = stage.drained();
    for(const char *name : named_at_p)
        CHECK(named(hidden, name) == nullptr);
}

TEST_CASE("the Linearization window's Show switch starts off and shows then hides the planes at the point on a surface", "[optimization][surface][linearization]")
{
    linearized_surface viewed;
    threepp::Scene &scene = *viewed.built.scene;
    CHECK_FALSE(drawn_visible(scene, "tangent plane at p"));
    CHECK_FALSE(drawn_visible(scene, "central difference at p"));
    CHECK(drawn_visible(scene, "surface"));
    CHECK(drawn_visible(scene, "tangent plane"));
    viewed.show();
    CHECK(drawn_visible(scene, "tangent plane at p"));
    CHECK(drawn_visible(scene, "central difference at p"));
    viewed.show();
    CHECK_FALSE(drawn_visible(scene, "tangent plane at p"));
    CHECK_FALSE(drawn_visible(scene, "central difference at p"));
}

TEST_CASE("a surface draws its planes at the point and no object for the point itself", "[optimization][surface][linearization]")
{
    linearized_surface viewed;
    std::vector<std::string> at_p;
    viewed.built.scene->traverse(
            [&at_p](threepp::Object3D &object)
            {
                if(object.name == "p" || object.name.ends_with(" p"))
                    at_p.push_back(object.name);
            });

    std::ranges::sort(at_p);
    CHECK(at_p == std::vector<std::string>{"central difference at p", "tangent plane at p"});
}

TEST_CASE("the Linearization window's Reset asks for the middle of the ranges the latest publication carries", "[optimization][linearization]")
{
    std::shared_ptr<scene::imgui_window> panel;
    graph_stage stage(cubic(), graph_and_linearization(panel), false);
    {
        tests::imgui_frame frames;
        type_into(frames, [panel] { panel->render(); }, 0, "2.5");
    }
    REQUIRE(stage.loop.drain().has_value());
    const std::shared_ptr<const scenario_snapshot> linearized = stage.taken.inputs->seen.read();
    REQUIRE((linearized != nullptr && linearized->linearization.has_value()));
    REQUIRE(linearized->linearization->point == Eigen::VectorXd::Constant(1, 2.5));
    CHECK(reset_point(panel, stage.loop, stage.taken) == Eigen::VectorXd::Constant(1, 0.0));

    command(stage.taken.inputs->scenario, [](scenario_state &state) { state.set_range(std::vector<interval>{interval{0.0, 6.0}}); });
    REQUIRE(stage.loop.drain().has_value());
    CHECK(reset_point(panel, stage.loop, stage.taken) == Eigen::VectorXd::Constant(1, 3.0));
}

TEST_CASE("over a surface the Linearization window's Reset asks for the middle of both ranges", "[optimization][surface][linearization]")
{
    linearized_surface viewed({interval{-2.0, 2.0}, interval{0.0, 4.0}});
    CHECK(reset_point(viewed.panel, viewed.loop, viewed.taken) == Eigen::VectorXd(Eigen::Vector2d(0.0, 2.0)));
}
