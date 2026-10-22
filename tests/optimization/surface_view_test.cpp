#include "drawn_surface.h"
#include "composed_curve.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/display_window.h"
#include "praxis/optimization/scenario_snapshot.h"

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

// A surface in a box, run from a seed off the zero curve, with the Display window kept when composed.
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
            , taken(open_surface_with(built.site, {boxed()}, baseline(),
                                      scenario_settings{"circle", {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, Eigen::Vector2d(1.5, 0.5), 70u, 0u, {}, {}}, composer(graphed)))
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
                graph = std::make_shared<display_window>("Display", offered);

            return graphed ? std::vector<std::shared_ptr<scene::imgui_window>>{graph} : std::vector<std::shared_ptr<scene::imgui_window>>{};
        };
    }

    void press(const char *label)
    {
        tests::imgui_frame frames;
        press_on(frames, [shown = graph] { shown->render(); }, "Display", label);
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
    const placement standing = placement_of(field_of(scene), *viewed.taken.inputs->seen.read());

    const std::vector<Eigen::Vector3d> sides = drawn_points(scene, "box");
    REQUIRE(sides.size() == 8u);
    CHECK(drawn_visible(scene, "box"));
    for(const Eigen::Vector3d &corner : {standing.at(-2.0, -1.5, 0.0), standing.at(1.2, -1.5, 0.0), standing.at(1.2, 2.0, 0.0), standing.at(-2.0, 2.0, 0.0)})
        CHECK(holds(sides, corner));
    for(const Eigen::Vector3d &end : sides)
        CHECK(std::abs(end.z()) <= 1.0e-6);
}

TEST_CASE("with no Display window the zero curve, the selected step's construction and the box are all drawn", "[optimization][surface][view]")
{
    viewed_surface viewed(false);
    threepp::Scene &scene = *viewed.built.scene;

    CHECK(drawn_visible(scene, "zero curve"));
    CHECK(drawn_visible(scene, "tangent plane"));
    CHECK(drawn_visible(scene, "zero line"));
    CHECK(drawn_visible(scene, "box"));

    const placement standing = placement_of(field_of(scene), *viewed.taken.inputs->seen.read());
    for(const Eigen::Vector3d &end : drawn_points(scene, "zero curve"))
        CHECK(std::abs(standing.over(end).norm() - 1.0) <= 1.0e-3);
}

TEST_CASE("the Display window's switches hide and show the zero curve, the selected step's construction and the box", "[optimization][surface][view]")
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
    for(const char *name : {"zero curve", "tangent plane", "nearest point", "zero line", "box"})
        CHECK(drawn_visible(scene, name));
}
