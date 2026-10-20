#include "drawn_surface.h"
#include "composed_curve.h"

#include "panel_keys.h"
#include "imgui_frame.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_snapshot.h"
#include "praxis/optimization/linearization_window.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr std::size_t x_row        = 0;
constexpr std::size_t y_row        = 2;
constexpr std::size_t y_slider_row = 3;

double circle_value(double x, double y)
{
    return x * x + y * y - 1.0;
}

Eigen::Vector2d circle_gradient(double x, double y)
{
    return Eigen::Vector2d(2.0 * x, 2.0 * y);
}

Eigen::Vector2d swapped_gradient(double x, double y)
{
    return Eigen::Vector2d(2.0 * y, 2.0 * x);
}

// A surface with the Linearization window, run from a seed on the zero curve so the step draws nothing.
struct linearized_surface
{
    scheduler::scheduler loop;
    staged built;
    std::shared_ptr<scene::imgui_window> panel;
    opened taken;

    linearized_surface(Eigen::Vector2d (*gradient)(double, double), bool checks_derivative)
            : loop(scheduler::inline_workers)
            , built(loop)
            , panel()
            , taken(open_surface_with(
                      built.site, {registered_surface("circle", &circle_value, gradient)}, baseline(),
                      scenario_settings{"circle", {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, Eigen::Vector2d(1.0, 0.0), 70u, 0u},
                      [this](const scenario_window_inputs &offered)
                      {
                          panel = std::make_shared<linearization_window>("Linearization", offered);
                          return std::vector<std::shared_ptr<scene::imgui_window>>{panel};
                      },
                      checks_derivative))
    {
        static_cast<void>(settled(loop, taken));
    }

    ~linearized_surface()
    {
        taken.composed->tear_down();
    }

    drawing draw() const
    {
        return [shown = panel] { shown->render(); };
    }

    const scenario_linearization &at()
    {
        REQUIRE(loop.drain().has_value());
        taken.composed->stencil->render();
        const std::shared_ptr<const scenario_snapshot> shown = taken.inputs->seen.read();
        REQUIRE(shown->linearization.has_value());

        return *shown->linearization;
    }

    void type_point(const char *x, const char *y)
    {
        tests::imgui_frame frames;
        type_into(frames, draw(), x_row, x);
        type_into(frames, draw(), y_row, y);
    }
};

// Every vertex of the drawn plane stands on z = f + g . (q - p), mapped back through the field's corners.
void check_plane(threepp::Scene &scene, const char *name, const Eigen::Vector2d &p, double f, const Eigen::Vector2d &gradient, const scenario_grid &grid)
{
    const placement standing                   = placement_of(field_of(scene), grid);
    const std::vector<Eigen::Vector3d> corners = drawn_points(scene, name);
    REQUIRE(corners.size() >= 3u);
    for(const Eigen::Vector3d &corner : corners)
    {
        const Eigen::Vector2d q((corner.x() - standing.origin.x()) / standing.across.x() - 2.0, (corner.y() - standing.origin.y()) / standing.across.y() - 2.0);
        CHECK(std::abs(corner.z() - standing.scale * (f + gradient.dot(q - p))) <= 1.0e-4);
    }
}

}

TEST_CASE("a point typed into both fields is evaluated, its value and gradient shown and its tangent plane drawn", "[optimization][surface][linearization]")
{
    linearized_surface staged_surface(&circle_gradient, false);
    CHECK(staged_surface.at().point == Eigen::Vector2d(0.0, 0.0));
    CHECK(drawn_visible(*staged_surface.built.scene, "tangent plane at p"));

    staged_surface.type_point("1.5", "0.5");
    const scenario_linearization &at = staged_surface.at();
    CHECK(at.point == Eigen::Vector2d(1.5, 0.5));

    const std::vector<std::string> lines = linearization_lines(at, false);
    REQUIRE(lines.size() == 2u);
    CHECK_THAT(lines[0], Catch::Matchers::ContainsSubstring("f(x, y) = 1.5"));
    CHECK_THAT(lines[1], Catch::Matchers::ContainsSubstring("(3, 1)"));

    threepp::Scene &scene = *staged_surface.built.scene;
    CHECK(drawn_visible(scene, "tangent plane at p"));
    CHECK_FALSE(drawn_visible(scene, "central difference at p"));
    check_plane(scene, "tangent plane at p", Eigen::Vector2d(1.5, 0.5), 1.5, Eigen::Vector2d(3.0, 1.0), *staged_surface.taken.inputs->seen.read()->grid);
}

TEST_CASE("the y slider moves the point along its range and leaves x where it was", "[optimization][surface][linearization]")
{
    linearized_surface staged_surface(&circle_gradient, false);
    staged_surface.type_point("1.5", "0.5");
    static_cast<void>(staged_surface.at());

    tests::imgui_frame frames;
    tweak_from(frames, staged_surface.draw(), y_slider_row, 3u);
    const scenario_linearization &at = staged_surface.at();
    CHECK(at.point[0] == 1.5);
    CHECK(at.point[1] != 0.5);
    CHECK((at.point[1] >= -2.0 && at.point[1] <= 2.0));
    CHECK(-(*at.error)[0] == circle_value(at.point[0], at.point[1]));
}

TEST_CASE("with the check opted in, a gradient with its components swapped is stated and both planes are drawn", "[optimization][surface][linearization]")
{
    linearized_surface staged_surface(&swapped_gradient, true);
    staged_surface.type_point("1.5", "0.5");
    const scenario_linearization &at = staged_surface.at();
    REQUIRE(at.check.has_value());
    CHECK(at.check->disagrees);
    CHECK((at.check->central.row(0).transpose() - Eigen::Vector2d(3.0, 1.0)).norm() <= 1.0e-6);

    const std::vector<std::string> lines = linearization_lines(at, true);
    REQUIRE(lines.size() == 4u);
    CHECK_THAT(lines[3], Catch::Matchers::ContainsSubstring("The gradient handed in and the central difference differ at (x, y) = (1.5, 0.5)"));

    threepp::Scene &scene = *staged_surface.built.scene;
    CHECK(drawn_visible(scene, "tangent plane at p"));
    REQUIRE(drawn_visible(scene, "central difference at p"));
    check_plane(scene, "central difference at p", Eigen::Vector2d(1.5, 0.5), 1.5, at.check->central.row(0).transpose(), *staged_surface.taken.inputs->seen.read()->grid);
}

TEST_CASE("with the check opted in, a correct gradient raises nothing and only the registered plane is drawn", "[optimization][surface][linearization]")
{
    linearized_surface staged_surface(&circle_gradient, true);
    staged_surface.type_point("1.5", "0.5");
    const scenario_linearization &at = staged_surface.at();
    REQUIRE(at.check.has_value());
    CHECK_FALSE(at.check->disagrees);
    CHECK(linearization_lines(at, true).size() == 3u);
    CHECK(drawn_visible(*staged_surface.built.scene, "tangent plane at p"));
    CHECK_FALSE(drawn_visible(*staged_surface.built.scene, "central difference at p"));
}

TEST_CASE("with the check not opted in, nothing of it is evaluated, written or drawn on a surface", "[optimization][surface][linearization]")
{
    linearized_surface staged_surface(&swapped_gradient, false);
    staged_surface.type_point("1.5", "0.5");
    const scenario_linearization &at = staged_surface.at();

    CHECK_FALSE(at.check.has_value());
    CHECK(linearization_lines(at, false).size() == 2u);
    CHECK(linearization_lines(at, true).size() == 2u);
    CHECK_FALSE(drawn_visible(*staged_surface.built.scene, "central difference at p"));
}
