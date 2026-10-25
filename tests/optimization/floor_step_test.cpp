#include "drawn_surface.h"
#include "minimized_surface.h"
#include "scenario_defaults.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/problem.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_view.h"
#include "praxis/optimization/surface_stencil.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/key_window.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/math/Vector3.hpp>

#include <threepp/objects/Mesh.hpp>

#include <threepp/materials/interfaces.hpp>

#include <threepp/geometries/CylinderGeometry.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <utility>

using namespace praxis;
using namespace praxis::fixture;

namespace {

struct drawn_rod
{
    Eigen::Vector3d from;
    Eigen::Vector3d to;
    double radius;
    threepp::Color tone;
};

// The rod's ends are its axis's ends: its center less and plus half its stretched height along its turned +y.
drawn_rod rod_of(threepp::Scene &scene)
{
    auto *rod = scene.getObjectByName<threepp::Mesh>("step taken");
    REQUIRE(rod != nullptr);
    const auto *shape = dynamic_cast<const threepp::CylinderGeometry *>(rod->geometry().get());
    const auto *worn  = dynamic_cast<const threepp::MaterialWithColor *>(rod->material().get());
    REQUIRE(shape != nullptr);
    REQUIRE(worn != nullptr);
    REQUIRE(shape->radiusTop == shape->radiusBottom);
    REQUIRE(rod->scale.x == rod->scale.z);

    threepp::Vector3 along(0.f, 1.f, 0.f);
    along.applyQuaternion(rod->quaternion);
    const Eigen::Vector3d center(rod->position.x, rod->position.y, rod->position.z);
    const Eigen::Vector3d half = Eigen::Vector3d(along.x, along.y, along.z) * (rod->scale.y * shape->height / 2.0);

    return drawn_rod{center - half, center + half, shape->radiusTop * rod->scale.x, worn->color};
}

// The floor projections of x_k and x_(k+1), raised to the rod's axis.
std::pair<Eigen::Vector3d, Eigen::Vector3d> expected_ends(threepp::Scene &scene, const scenario_snapshot &shown)
{
    const placement standing = placement_of(field_of(scene), shown);
    const Eigen::Vector3d raised(0.0, 0.0, drawn_points(scene, "level curves").front().z() + floor_step_lift);
    const Eigen::VectorXd &x    = shown.run.iterates[shown.step];
    const Eigen::VectorXd &next = shown.run.iterates[shown.step + 1u];

    return {standing.at(x[0], x[1], 0.0) + raised, standing.at(next[0], next[1], 0.0) + raised};
}

void check_ends(threepp::Scene &scene, const scenario_snapshot &shown)
{
    const drawn_rod rod                                  = rod_of(scene);
    const std::pair<Eigen::Vector3d, Eigen::Vector3d> at = expected_ends(scene, shown);
    CHECK((rod.from - at.first).norm() <= 1.0e-5);
    CHECK((rod.to - at.second).norm() <= 1.0e-5);
}

// Steps from x_0 out of the ranges, so the step from x_0 has an answered gradient and no end to draw to.
expected<Eigen::VectorXd, refusal> leaving(const problem &, const Eigen::VectorXd &x0, double, std::uint32_t, std::vector<Eigen::VectorXd> &iterates)
{
    iterates = {x0, Eigen::Vector2d(3.0, 0.0), Eigen::Vector2d(0.5, 0.0)};

    return unexpected(refusal::no_solution);
}

scenario_settings descended(const Eigen::Vector2d &seed)
{
    return scenario_settings{
            "(x^2 + 10y^2)/2", {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, seed, 500u, 0u, objective_settings{problem_form::minimum, minimizer_slot::gradient_descent, 0.1}, {}};
}

}

TEST_CASE("with Floor projection on, the step taken over a minimized surface is a rod from x_k's floor projection to x_(k+1)'s", "[optimization][surface][objective]")
{
    minimized_surface drawn(problem_form::minimum);
    drawn.press("Floor projection");
    REQUIRE(drawn.shown()->step == 0u);
    REQUIRE(drawn_visible(drawn.scene(), "step taken"));

    const drawn_rod rod = rod_of(drawn.scene());
    CHECK(std::abs(rod.radius - floor_step_radius) <= 1.0e-7);
    CHECK(rod.tone == threepp::Color(floor_step_tone));
    check_ends(drawn.scene(), *drawn.shown());
}

TEST_CASE("the rod on the floor follows the selected step of a run", "[optimization][surface][objective]")
{
    minimized_surface drawn(elliptic(), descended(Eigen::Vector2d(-2.0, -2.0)));
    drawn.press("Floor projection");
    REQUIRE(drawn.shown()->run.iterates.size() > 7u);

    for(const std::uint32_t k : {5u, 1u})
    {
        drawn.select(k);
        REQUIRE(drawn.shown()->step == k);
        CHECK(drawn_visible(drawn.scene(), "step taken"));
        check_ends(drawn.scene(), *drawn.shown());
    }
}

TEST_CASE("a step to an x_(k+1) outside the ranges draws no rod", "[optimization][surface][objective]")
{
    capabilities solving               = baseline();
    solving.minimizer.gradient_descent = &leaving;
    minimized_surface drawn(elliptic(), descended(Eigen::Vector2d(0.5, 0.5)), solving);
    drawn.press("Floor projection");
    REQUIRE(drawn.shown()->step == 0u);
    REQUIRE(drawn.shown()->run.gradients[0].has_value());

    CHECK(drawn_visible(drawn.scene(), "gradient at x_k"));
    CHECK_FALSE(drawn_visible(drawn.scene(), "step taken"));

    drawn.select(1u);
    REQUIRE(drawn.shown()->step == 1u);
    CHECK_FALSE(drawn_visible(drawn.scene(), "step taken"));
}

TEST_CASE("the rod is 0.01 m in radius and its axis stands 0.025 m above the floor", "[optimization][surface][objective]")
{
    CHECK(std::abs(floor_step_radius - 0.01) <= 1.0e-12);
    CHECK(std::abs(floor_step_lift - 0.025) <= 1.0e-12);

    minimized_surface drawn(problem_form::minimum);
    drawn.press("Floor projection");
    const drawn_rod rod = rod_of(drawn.scene());
    const double floor  = drawn_points(drawn.scene(), "level curves").front().z();
    CHECK(std::abs(rod.radius - 0.01) <= 1.0e-7);
    CHECK(std::abs(rod.from.z() - floor - 0.025) <= 1.0e-6);
    CHECK(std::abs(rod.to.z() - floor - 0.025) <= 1.0e-6);
}

TEST_CASE("a surface with nothing published draws no level map on its floor", "[optimization][surface][objective]")
{
    threepp::Scene scene;
    const auto published = std::make_shared<scenario_publisher>();
    const auto keyed     = std::make_shared<std::vector<scene::key_entry>>();
    surface_stencil drawn(scene, published->reader(), std::make_shared<scenario_view>(), keyed);
    REQUIRE(drawn.initialize().has_value());
    drawn.render();

    CHECK_FALSE(drawn_visible(scene, "level map"));
    CHECK_FALSE(drawn_visible(scene, "gradient at x_k"));
    CHECK_FALSE(drawn_visible(scene, "step taken"));
    CHECK(keyed->empty());
    drawn.tear_down();
}
