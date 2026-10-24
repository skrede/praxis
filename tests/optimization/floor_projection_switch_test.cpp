#include "drawn_surface.h"
#include "surface_levels.h"
#include "minimized_surface.h"
#include "scenario_defaults.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/scenario_snapshot.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/math/Vector3.hpp>

#include <threepp/objects/Mesh.hpp>

#include <threepp/materials/Material.hpp>

#include <threepp/geometries/CylinderGeometry.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;

namespace {

// The center of x_k's marker on the field, in the field's frame.
Eigen::Vector3d marker_of(threepp::Scene &scene, const scenario_snapshot &shown, std::size_t k)
{
    REQUIRE(k < shown.run.errors.size());
    REQUIRE(shown.run.errors[k].has_value());
    const Eigen::VectorXd &x = shown.run.iterates[k];

    return placement_of(field_of(scene), shown).at(x[0], x[1], -(*shown.run.errors[k])[0]);
}

// The rod's axis ends: its center less and plus half its stretched height along its turned +y.
std::pair<Eigen::Vector3d, Eigen::Vector3d> rod_ends(threepp::Scene &scene)
{
    auto *rod = scene.getObjectByName<threepp::Mesh>("step taken");
    REQUIRE(rod != nullptr);
    const auto *shape = dynamic_cast<const threepp::CylinderGeometry *>(rod->geometry().get());
    REQUIRE(shape != nullptr);

    threepp::Vector3 along(0.f, 1.f, 0.f);
    along.applyQuaternion(rod->quaternion);
    const Eigen::Vector3d center(rod->position.x, rod->position.y, rod->position.z);
    const Eigen::Vector3d half = Eigen::Vector3d(along.x, along.y, along.z) * (rod->scale.y * shape->height / 2.0);

    return {center - half, center + half};
}

Eigen::Vector3d gradient_direction(threepp::Scene &scene, const scenario_snapshot &shown)
{
    const placement standing = placement_of(field_of(scene), shown);
    const Eigen::VectorXd &x = shown.run.iterates[shown.step];
    const Eigen::VectorXd &g = *shown.run.gradients[shown.step];

    return (standing.at(x[0] + g[0], x[1] + g[1], 0.0) - standing.at(x[0], x[1], 0.0)).normalized();
}

// Every written point stands at the field's height of one of the map's levels, above the field's
// lowest drawn vertex, and the points stand at two levels or more.
void check_curves_on_field(threepp::Scene &scene, const scenario_snapshot &shown)
{
    REQUIRE(drawn_visible(scene, "level curves"));
    const drawn_field field          = field_of(scene);
    const double scale               = placement_of(field, shown).scale;
    const double lowest              = lowest_drawn(field);
    const std::vector<double> levels = map_levels(*shown.grid);
    std::vector<double> met;
    std::size_t missed = 0;
    std::size_t sunk   = 0;
    for(const Eigen::Vector3d &point : drawn_points(scene, "level curves"))
    {
        const auto level = std::ranges::find_if(levels, [&](double c) { return std::abs(point.z() - scale * c) <= 1.0e-6; });
        missed += level == levels.end() ? 1u : 0u;
        sunk += point.z() <= lowest ? 1u : 0u;
        if(level != levels.end() && std::ranges::find(met, *level) == met.end())
            met.push_back(*level);
    }
    CHECK(missed == 0u);
    CHECK(sunk == 0u);
    CHECK(met.size() >= 2u);
}

void check_figures_on_field(threepp::Scene &scene, const scenario_snapshot &shown)
{
    const std::size_t k = shown.step;
    CHECK_FALSE(drawn_visible(scene, "iterates on the floor"));
    REQUIRE(drawn_visible(scene, "gradient at x_k"));
    CHECK((drawn_position(scene, "gradient at x_k") - marker_of(scene, shown, k)).norm() <= 1.0e-5);
    CHECK((drawn_position(scene, "gradient at x_k") - shown_markers(scene).back()).norm() <= 1.0e-5);
    CHECK((pointing(scene) - gradient_direction(scene, shown)).norm() <= 1.0e-5);

    REQUIRE(drawn_visible(scene, "step taken"));
    const std::pair<Eigen::Vector3d, Eigen::Vector3d> rod = rod_ends(scene);
    CHECK((rod.first - marker_of(scene, shown, k)).norm() <= 1.0e-5);
    CHECK((rod.second - marker_of(scene, shown, k + 1u)).norm() <= 1.0e-5);
}

void check_on_floor(threepp::Scene &scene, const scenario_snapshot &shown)
{
    const std::vector<Eigen::Vector3d> curve = drawn_points(scene, "level curves");
    REQUIRE(drawn_visible(scene, "level curves"));
    REQUIRE_FALSE(curve.empty());
    const double floor = curve.front().z();
    CHECK(floor < lowest_drawn(field_of(scene)));
    CHECK(std::ranges::all_of(curve, [floor](const Eigen::Vector3d &point) { return std::abs(point.z() - floor) <= 1.0e-6; }));
    CHECK(drawn_visible(scene, "iterates on the floor"));
    CHECK(projected_centers(scene).size() == shown_markers(scene).size());

    const placement standing = placement_of(field_of(scene), shown);
    const Eigen::VectorXd &x = shown.run.iterates[shown.step];
    const Eigen::Vector3d at = standing.at(x[0], x[1], 0.0) + Eigen::Vector3d(0.0, 0.0, floor);
    CHECK((drawn_position(scene, "gradient at x_k") - at).norm() <= 1.0e-5);
    CHECK((pointing(scene) - gradient_direction(scene, shown)).norm() <= 1.0e-5);
    CHECK(std::abs(rod_ends(scene).first.z() - floor - floor_step_lift) <= 1.0e-6);
    CHECK((rod_ends(scene).first.head<2>() - at.head<2>()).norm() <= 1.0e-5);
}

// Every part of the named figure drawn over the field, after it and with no depth test or write, or
// every part depth-tested.
bool laid_over(threepp::Scene &scene, const char *name, bool over)
{
    threepp::Object3D *found = scene.getObjectByName(name);
    REQUIRE(found != nullptr);
    bool all = true;
    found->traverse(
            [over, &all](threepp::Object3D &part)
            {
                const std::shared_ptr<threepp::Material> tone = part.material();
                if(tone != nullptr)
                    all = all && tone->depthTest != over && tone->depthWrite != over && tone->transparent == over && (part.renderOrder > 0) == over;
            });

    return all;
}

void check_modes(threepp::Scene &scene, bool on_field)
{
    CHECK(laid_over(scene, "level curves", false));
    CHECK(laid_over(scene, "gradient at x_k", on_field));
    CHECK(laid_over(scene, "step taken", on_field));
}

scenario_settings descended()
{
    return scenario_settings{"(x^2 + 10y^2)/2",
                             {interval{-2.0, 2.0}, interval{-2.0, 2.0}},
                             Eigen::Vector2d(-2.0, -2.0),
                             500u,
                             0u,
                             objective_settings{problem_form::minimum, minimizer_slot::gradient_descent, 0.1},
                             {}};
}

}

TEST_CASE("a minimized surface opens with its level curves, its gradient and its step on the field", "[optimization][surface][objective][view]")
{
    minimized_surface drawn(problem_form::minimum);
    REQUIRE(drawn.shown()->step == 0u);
    CHECK(drawn.offered().find("Floor projection") != std::string::npos);

    check_curves_on_field(drawn.scene(), *drawn.shown());
    check_figures_on_field(drawn.scene(), *drawn.shown());
}

TEST_CASE("the Floor projection switch moves the level map to the floor and back", "[optimization][surface][objective][view]")
{
    minimized_surface drawn(problem_form::minimum);
    drawn.press("Floor projection");
    check_on_floor(drawn.scene(), *drawn.shown());

    drawn.press("Floor projection");
    check_curves_on_field(drawn.scene(), *drawn.shown());
    check_figures_on_field(drawn.scene(), *drawn.shown());
}

TEST_CASE("the gradient and the step on the field follow the selected step", "[optimization][surface][objective][view]")
{
    minimized_surface drawn(elliptic(), descended());
    REQUIRE(drawn.shown()->run.iterates.size() > 7u);
    for(const std::uint32_t k : {5u, 1u})
    {
        drawn.select(k);
        REQUIRE(drawn.shown()->step == k);
        check_figures_on_field(drawn.scene(), *drawn.shown());
    }
}

TEST_CASE("on the field the gradient and the step are drawn over the field and the curves depth-tested at their levels; on the floor all three are depth-tested",
          "[optimization][surface][objective][view]")
{
    minimized_surface drawn(problem_form::minimum);
    check_modes(drawn.scene(), true);
    drawn.press("Floor projection");
    check_modes(drawn.scene(), false);
    drawn.press("Floor projection");
    check_modes(drawn.scene(), true);
}

TEST_CASE("a root-form surface offers no Floor projection and keeps its zero curve at zero", "[optimization][surface][objective][view]")
{
    const registered_surface circle("x^2 + y^2 - 1", [](double x, double y) { return x * x + y * y - 1.0; }, [](double x, double y) { return Eigen::Vector2d(2.0 * x, 2.0 * y); });
    minimized_surface drawn(circle,
                            scenario_settings{"x^2 + y^2 - 1",
                                              {interval{-2.0, 2.0}, interval{-2.0, 2.0}},
                                              Eigen::Vector2d(1.5, 1.5),
                                              500u,
                                              0u,
                                              objective_settings{problem_form::root, minimizer_slot::newton, 0.1},
                                              {}});
    CHECK(drawn.offered().find("Floor projection") == std::string::npos);

    REQUIRE(drawn_visible(drawn.scene(), "zero curve"));
    const std::vector<Eigen::Vector3d> curve = drawn_points(drawn.scene(), "zero curve");
    REQUIRE_FALSE(curve.empty());
    CHECK(std::ranges::all_of(curve, [](const Eigen::Vector3d &point) { return std::abs(point.z()) <= 1.0e-6; }));
}
