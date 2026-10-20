#include "drawn_surface.h"
#include "composed_curve.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"
#include "praxis/optimization/surface_graph_window.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr double unbounded = std::numeric_limits<double>::infinity();

double circle_value(double x, double y)
{
    return x * x + y * y - 1.0;
}

Eigen::Vector2d circle_gradient(double x, double y)
{
    return Eigen::Vector2d(2.0 * x, 2.0 * y);
}

registered_surface boxed()
{
    return registered_surface("circle", &circle_value, &circle_gradient, box{Eigen::Vector2d(-unbounded, -1.5), Eigen::Vector2d(1.2, unbounded)});
}

// A surface in a box, run from a seed off the zero curve, with the graph window kept when composed.
struct viewed_surface
{
    scheduler::scheduler loop;
    staged built;
    std::shared_ptr<scene::imgui_window> graph;
    opened taken;

    explicit viewed_surface(bool graphed)
            : loop(scheduler::inline_workers)
            , built(loop)
            , graph()
            , taken(open_surface_with(built.site, {boxed()}, baseline(), scenario_settings{"circle", {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, Eigen::Vector2d(1.5, 0.5), 70u, 0u},
                                      composer(graphed)))
    {
        static_cast<void>(settled(loop, taken));
        taken.composed->stencil->render();
    }

    ~viewed_surface()
    {
        taken.composed->tear_down();
    }

    scenario_window_composer composer(bool graphed)
    {
        return [this, graphed](const scenario_window_inputs &offered)
        {
            if(graphed)
                graph = std::make_shared<surface_graph_window>("Graph", offered);

            return graphed ? std::vector<std::shared_ptr<scene::imgui_window>>{graph} : std::vector<std::shared_ptr<scene::imgui_window>>{};
        };
    }

    void press(const char *label)
    {
        tests::imgui_frame frames;
        press_on(frames, [shown = graph] { shown->render(); }, "Graph", label);
        taken.composed->stencil->render();
    }
};

bool holds(const std::vector<Eigen::Vector3d> &drawn, const Eigen::Vector3d &expected)
{
    return std::ranges::any_of(drawn, [&expected](const Eigen::Vector3d &point) { return (point - expected).norm() <= 1.0e-5; });
}

}

TEST_CASE("a surface in a box draws the box on the zero plane, an infinite side at the range's edge", "[optimization][surface][view]")
{
    viewed_surface viewed(false);
    threepp::Scene &scene    = *viewed.built.scene;
    const placement standing = placement_of(field_of(scene), *viewed.taken.inputs->seen.read()->grid);

    const std::vector<Eigen::Vector3d> sides = drawn_points(scene, "box");
    REQUIRE(sides.size() == 8u);
    CHECK(drawn_visible(scene, "box"));
    for(const Eigen::Vector3d &corner : {standing.at(-2.0, -1.5, 0.0), standing.at(1.2, -1.5, 0.0), standing.at(1.2, 2.0, 0.0), standing.at(-2.0, 2.0, 0.0)})
        CHECK(holds(sides, corner));
    for(const Eigen::Vector3d &end : sides)
        CHECK(std::abs(end.z()) <= 1.0e-6);
}

TEST_CASE("with no graph window the zero curve, the selected step's construction and the box are all drawn", "[optimization][surface][view]")
{
    viewed_surface viewed(false);
    threepp::Scene &scene = *viewed.built.scene;

    CHECK(drawn_visible(scene, "zero curve"));
    CHECK(drawn_visible(scene, "tangent plane"));
    CHECK(drawn_visible(scene, "zero line"));
    CHECK(drawn_visible(scene, "box"));

    const placement standing = placement_of(field_of(scene), *viewed.taken.inputs->seen.read()->grid);
    for(const Eigen::Vector3d &end : drawn_points(scene, "zero curve"))
        CHECK(std::abs(Eigen::Vector2d((end.x() - standing.origin.x()) / standing.across.x() - 2.0, (end.y() - standing.origin.y()) / standing.across.y() - 2.0).norm() - 1.0) <=
              1.0e-3);
}

TEST_CASE("the graph window's switches hide and show the zero curve, the selected step's construction and the box", "[optimization][surface][view]")
{
    viewed_surface viewed(true);
    threepp::Scene &scene = *viewed.built.scene;

    viewed.press("Zero curve");
    CHECK_FALSE(drawn_visible(scene, "zero curve"));
    CHECK(drawn_visible(scene, "tangent plane"));

    viewed.press("Step");
    CHECK_FALSE(drawn_visible(scene, "tangent plane"));
    CHECK_FALSE(drawn_visible(scene, "nearest point"));
    CHECK(drawn_visible(scene, "box"));
    CHECK(drawn_visible(scene, "surface"));

    viewed.press("Box");
    CHECK_FALSE(drawn_visible(scene, "box"));

    for(const char *label : {"Zero curve", "Step", "Box"})
        viewed.press(label);
    for(const char *name : {"zero curve", "tangent plane", "box"})
        CHECK(drawn_visible(scene, name));
}

TEST_CASE("the graph window's linearization switch hides and shows the plane at the linearized point alone", "[optimization][surface][view]")
{
    viewed_surface viewed(true);
    threepp::Scene &scene = *viewed.built.scene;
    command(viewed.taken.inputs->scenario, [](scenario_state &state) { state.linearize(Eigen::Vector2d(0.5, 0.5)); });
    REQUIRE(viewed.loop.drain().has_value());
    viewed.taken.composed->stencil->render();
    REQUIRE(drawn_visible(scene, "tangent plane at p"));

    viewed.press("Linearization");
    CHECK_FALSE(drawn_visible(scene, "tangent plane at p"));
    CHECK(drawn_visible(scene, "tangent plane"));

    viewed.press("Linearization");
    CHECK(drawn_visible(scene, "tangent plane at p"));
}
