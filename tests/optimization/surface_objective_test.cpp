#include "drawn_surface.h"
#include "minimized_surface.h"
#include "scenario_defaults.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;

namespace {

double drawn_length(threepp::Scene &scene)
{
    threepp::Object3D *arrow = scene.getObjectByName("gradient at x_k");
    REQUIRE(arrow != nullptr);
    threepp::Object3D *shaft = arrow->getObjectByName("shaft");
    threepp::Object3D *head  = arrow->getObjectByName("head");

    return (shaft->visible ? shaft->scale.y : 0.0) + head->scale.y * gradient_head_length;
}

// Undefined beyond x = 1, so f and its gradient are refused there.
registered_surface cut_off()
{
    const double undefined = std::numeric_limits<double>::quiet_NaN();
    return registered_surface(
            "cut off", [undefined](double x, double y) { return x > 1.0 ? undefined : x * x + y * y; },
            [undefined](double x, double y) { return x > 1.0 ? Eigen::Vector2d(undefined, undefined) : Eigen::Vector2d(2.0 * x, 2.0 * y); });
}

// Steps past x = 1 and back, so the step from x_1 is selectable while its gradient is refused.
expected<Eigen::VectorXd, refusal> leaping(const problem &, const Eigen::VectorXd &x0, double, std::uint32_t, std::vector<Eigen::VectorXd> &iterates)
{
    iterates = {x0, Eigen::Vector2d(1.5, 0.0), Eigen::Vector2d(0.5, 0.0)};

    return unexpected(refusal::no_solution);
}

scenario_settings descended(std::string name, const Eigen::Vector2d &seed)
{
    return scenario_settings{
            std::move(name), {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, seed, 500u, 0u, objective_settings{problem_form::minimum, minimizer_slot::gradient_descent, 0.1}};
}

}

TEST_CASE("a minimized surface draws its level curves and its iterates on a floor plane under the field", "[optimization][surface][objective]")
{
    minimized_surface drawn(problem_form::minimum);
    threepp::Scene &scene                    = drawn.scene();
    const std::vector<Eigen::Vector3d> curve = drawn_points(scene, "level curves");
    REQUIRE(drawn_visible(scene, "level curves"));
    REQUIRE_FALSE(curve.empty());

    const double floor = curve.front().z();
    CHECK(floor < lowest_drawn(field_of(scene)));
    for(const Eigen::Vector3d &point : curve)
        CHECK(std::abs(point.z() - floor) <= 1.0e-6);

    const std::vector<Eigen::Vector3d> above = shown_markers(scene);
    const std::vector<Eigen::Vector3d> below = projected_centers(scene);
    REQUIRE(below.size() == above.size());
    for(std::size_t k = 0; k < above.size(); ++k)
        CHECK((below[k] - Eigen::Vector3d(above[k].x(), above[k].y(), floor)).norm() <= 1.0e-5);
    CHECK_FALSE(drawn_visible(scene, "zero curve"));
    CHECK_FALSE(drawn_visible(scene, "tangent plane"));
}

TEST_CASE("a minimized surface draws the gradient at x_k and the step to x_(k+1) on that floor", "[optimization][surface][objective]")
{
    minimized_surface drawn(problem_form::minimum);
    threepp::Scene &scene          = drawn.scene();
    const scenario_snapshot &shown = *drawn.shown();
    const placement standing       = placement_of(field_of(scene), shown);
    const Eigen::Vector3d floor(0.0, 0.0, drawn_points(scene, "level curves").front().z());
    const Eigen::VectorXd &x    = shown.run.iterates[0];
    const Eigen::VectorXd &next = shown.run.iterates[1];
    const Eigen::VectorXd &g    = *shown.run.gradients[0];
    const Eigen::Vector3d at    = standing.at(x[0], x[1], 0.0) + floor;

    REQUIRE(drawn_visible(scene, "gradient at x_k"));
    CHECK((drawn_position(scene, "gradient at x_k") - at).norm() <= 1.0e-5);
    CHECK((pointing(scene) - (standing.at(x[0] + g[0], x[1] + g[1], 0.0) - standing.at(x[0], x[1], 0.0)).normalized()).norm() <= 1.0e-5);

    const std::vector<Eigen::Vector3d> step = drawn_points(scene, "step on the floor");
    REQUIRE(step.size() == 2u);
    CHECK(drawn_visible(scene, "step on the floor"));
    CHECK((step[0] - at).norm() <= 1.0e-5);
    CHECK((step[1] - (standing.at(next[0], next[1], 0.0) + floor)).norm() <= 1.0e-5);
}

TEST_CASE("a root-form run of the same surface draws no level map", "[optimization][surface][objective]")
{
    minimized_surface drawn(problem_form::root);
    threepp::Scene &scene = drawn.scene();

    CHECK_FALSE(drawn_visible(scene, "level curves"));
    CHECK_FALSE(drawn_visible(scene, "gradient at x_k"));
    CHECK_FALSE(drawn_visible(scene, "step on the floor"));
    CHECK(drawn_visible(scene, "surface"));
}

TEST_CASE("a step whose gradient was refused draws no arrow and no step on the floor", "[optimization][surface][objective]")
{
    capabilities solving               = baseline();
    solving.minimizer.gradient_descent = &leaping;
    minimized_surface drawn(cut_off(), descended("cut off", Eigen::Vector2d(0.5, 0.5)), solving);
    REQUIRE(drawn.shown()->run.gradients.size() == 3u);
    REQUIRE_FALSE(drawn.shown()->run.gradients[1].has_value());

    CHECK(drawn_visible(drawn.scene(), "gradient at x_k"));
    CHECK(drawn_visible(drawn.scene(), "step on the floor"));

    drawn.select(1u);
    REQUIRE(drawn.shown()->step == 1u);
    CHECK_FALSE(drawn_visible(drawn.scene(), "gradient at x_k"));
    CHECK_FALSE(drawn_visible(drawn.scene(), "step on the floor"));
    CHECK(drawn_visible(drawn.scene(), "level curves"));
}

TEST_CASE("a grid holding no value on a disc draws no level curve inside it", "[optimization][surface][objective]")
{
    const double undefined = std::numeric_limits<double>::quiet_NaN();
    const registered_surface holed("holed", [undefined](double x, double y) { return x * x + y * y < 0.25 ? undefined : x; }, [](double, double) { return Eigen::Vector2d(1.0, 0.0); });
    minimized_surface drawn(holed, descended("holed", Eigen::Vector2d(1.5, 1.5)));
    const placement standing = placement_of(field_of(drawn.scene()), *drawn.shown());

    const std::vector<Eigen::Vector3d> curve = drawn_points(drawn.scene(), "level curves");
    REQUIRE_FALSE(curve.empty());
    std::size_t beside = 0;
    double nearest     = std::numeric_limits<double>::infinity();
    for(const Eigen::Vector3d &point : curve)
    {
        const Eigen::Vector2d over = standing.over(point);
        nearest                    = std::min(nearest, over.norm());
        beside += std::abs(over.x()) < 0.5 ? 1u : 0u;
    }
    CHECK(beside > 0u);
    CHECK(nearest >= 0.5 - 1.0e-3);
}

TEST_CASE("the arrow at a converged run's last step is drawn at the arrow's length however small the gradient", "[optimization][surface][objective]")
{
    minimized_surface drawn(elliptic(), descended("(x^2 + 10y^2)/2", Eigen::Vector2d(-2.0, -2.0)));
    REQUIRE(drawn.shown()->run.answer.has_value());
    const std::size_t steps = drawn.shown()->run.iterates.size() - 1u;
    drawn.select(static_cast<std::uint32_t>(steps - 1u));

    const Eigen::VectorXd &g = *drawn.shown()->run.gradients[steps - 1u];
    REQUIRE(g.norm() < 1.0e-10);
    REQUIRE(drawn_visible(drawn.scene(), "gradient at x_k"));
    CHECK(std::abs(drawn_length(drawn.scene()) - gradient_arrow_fraction * surface_extent) <= 1.0e-6);
}
