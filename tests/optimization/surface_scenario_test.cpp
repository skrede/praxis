#include "drawn_surface.h"
#include "composed_curve.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/optimization/baseline/solver.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;

namespace {

double circle_value(double x, double y)
{
    return x * x + y * y - 1.0;
}

Eigen::Vector2d circle_gradient(double x, double y)
{
    return Eigen::Vector2d(2.0 * x, 2.0 * y);
}

// Undefined on the disc of radius 1/2 about the origin, which the solve from the seed never enters.
double holed_value(double x, double y)
{
    return x * x + y * y < 0.25 ? std::numeric_limits<double>::quiet_NaN() : circle_value(x, y);
}

scenario_settings surface_opening(std::string selected)
{
    return scenario_settings{std::move(selected), {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, Eigen::Vector2d(1.5, 0.5), 70u, 0u};
}

opened open_surface(const scene::preset_site &site, std::vector<registered_surface> surfaces, const scenario_settings &settings)
{
    opened taken;
    const scenario_window_composer kept = [&taken](const scenario_window_inputs &offered)
    {
        taken.inputs = offered;

        return std::vector<std::shared_ptr<scene::imgui_window>>{};
    };
    taken.composed = compose_surface_scenario(site, std::move(surfaces), baseline(), settings, scenario_composition{kept, false});

    return taken;
}

// Every sample of the circle's grid is a vertex at s f, and every iterate a marker on the field.
void check_circle_drawn(threepp::Scene &scene, const scenario_snapshot &ran)
{
    const std::size_t n      = ran.grid->count;
    const drawn_field field  = field_of(scene);
    const placement standing = placement_of(field, *ran.grid);
    CHECK(field.triangles == 2u * (n - 1u) * (n - 1u));
    for(std::size_t at = 0; at < n * n; at += 97u)
        CHECK(std::abs(vertex(field, at).z() - standing.scale * ran.grid->heights[at]) < 1.0e-6);

    const std::vector<Eigen::Vector3d> markers = shown_markers(scene);
    REQUIRE(markers.size() == ran.run.iterates.size());
    for(std::size_t k = 0; k < markers.size(); ++k)
    {
        const Eigen::VectorXd &x = ran.run.iterates[k];
        CHECK((markers[k] - standing.at(x[0], x[1], circle_value(x[0], x[1]))).norm() < 1.0e-5);
    }
}

}

TEST_CASE("a composed surface scenario publishes the run and the stencil draws the field and the iterates", "[optimization][surface]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);

    const opened taken                                 = open_surface(built.site, {registered_surface("circle", &circle_value, &circle_gradient)}, surface_opening("circle"));
    const std::shared_ptr<const scenario_snapshot> ran = settled(loop, taken);
    taken.composed->stencil->render();

    REQUIRE(ran != nullptr);
    CHECK(taken.inputs->dimension == 2u);
    REQUIRE(ran->run.iterates.size() >= 2u);
    CHECK(ran->run.iterates.front() == Eigen::Vector2d(1.5, 0.5));
    CHECK_FALSE(ran->run.refused.has_value());
    CHECK(std::abs(circle_value(ran->run.iterates.back()[0], ran->run.iterates.back()[1])) < 1.0e-10);
    REQUIRE(ran->grid != nullptr);

    check_circle_drawn(*built.scene, *ran);

    taken.composed->tear_down();
    CHECK(built.scene->getObjectByName("surface") == nullptr);
}

TEST_CASE("a surface undefined on a disc draws fewer triangles and none touching a sample it did not answer", "[optimization][surface]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);

    const opened taken                                 = open_surface(built.site, {registered_surface("holed", &holed_value, &circle_gradient)}, surface_opening("holed"));
    const std::shared_ptr<const scenario_snapshot> ran = settled(loop, taken);
    taken.composed->stencil->render();

    REQUIRE(ran != nullptr);
    REQUIRE(ran->grid != nullptr);
    CHECK_FALSE(ran->run.refused.has_value());

    const std::size_t n     = ran->grid->count;
    const drawn_field field = field_of(*built.scene);
    CHECK(field.triangles > 0u);
    CHECK(field.triangles < 2u * (n - 1u) * (n - 1u));
    for(const unsigned int corner : field.corners)
        CHECK(std::isfinite(ran->grid->heights[corner]));

    taken.composed->tear_down();
}

TEST_CASE("settings naming no registered surface, or other than two ranges and two seed coordinates, compose nothing", "[optimization][surface]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);

    scenario_settings narrowed = surface_opening("circle");
    narrowed.ranges.pop_back();
    scenario_settings unseeded = surface_opening("circle");
    unseeded.seed              = Eigen::VectorXd::Constant(1, 1.5);

    for(const scenario_settings &settings : {surface_opening("absent"), narrowed, unseeded})
    {
        const opened taken = open_surface(built.site, {registered_surface("circle", &circle_value, &circle_gradient)}, settings);
        CHECK(taken.composed == nullptr);
        CHECK_FALSE(taken.inputs.has_value());
    }
}

TEST_CASE("a shorter run leaves one marker in the scene per iterate it recorded", "[optimization][surface]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const opened taken                                 = open_surface(built.site, {registered_surface("circle", &circle_value, &circle_gradient)}, surface_opening("circle"));
    const std::shared_ptr<const scenario_snapshot> ran = settled(loop, taken);
    taken.composed->stencil->render();
    REQUIRE(ran->run.iterates.size() > 2u);

    command(taken.inputs->scenario, [](scenario_state &state) { state.run(Eigen::Vector2d(1.5, 0.5), 1u); });
    REQUIRE(loop.drain().has_value());
    taken.composed->stencil->render();

    CHECK(built.scene->getObjectByName("iterates")->children.size() == 2u);

    taken.composed->tear_down();
}
