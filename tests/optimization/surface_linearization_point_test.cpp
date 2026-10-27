#include "drawn_surface.h"
#include "keyed_surface.h"
#include "minimized_surface.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/scenario_view.h"
#include "praxis/optimization/scenario_snapshot.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/objects/Mesh.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/materials/Material.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <vector>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr const char *mark = "linearization point p";

double circle_value(double x, double y)
{
    return x * x + y * y - 1.0;
}

std::unique_ptr<keyed_surface> circle_surface()
{
    const scenario_settings settings{"circle", {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, Eigen::Vector2d(1.0, 0.0), 70u, 0u, {}, {}};

    return std::make_unique<keyed_surface>(registered_surface("circle", &circle_value, [](double x, double y) { return Eigen::Vector2d(2.0 * x, 2.0 * y); }), settings, false);
}

std::shared_ptr<const scenario_snapshot> published(keyed_surface &keyed)
{
    const std::shared_ptr<const scenario_snapshot> shown = keyed.taken.inputs->seen.read();
    REQUIRE(shown != nullptr);

    return shown;
}

// The mark stands at (p, f(p)) placed as the field is, and every corner of the plane on z = f(p) + g . (q - p).
void check_at(keyed_surface &keyed, const Eigen::Vector2d &p)
{
    threepp::Scene &scene    = keyed.scene();
    const placement standing = placement_of(field_of(scene), *published(keyed));
    const double f           = circle_value(p.x(), p.y());
    REQUIRE(drawn_visible(scene, mark));
    CHECK((drawn_position(scene, mark) - standing.at(p.x(), p.y(), f)).norm() <= 1.0e-4);

    const std::vector<Eigen::Vector3d> corners = drawn_points(scene, "tangent plane at p");
    REQUIRE(corners.size() >= 3u);
    for(const Eigen::Vector3d &corner : corners)
        CHECK(std::abs(corner.z() - standing.scale * (f + (2.0 * p).dot(standing.over(corner) - p))) <= 1.0e-4);
}

threepp::Material &material_of(threepp::Scene &scene, const char *name)
{
    threepp::Object3D *found = scene.getObjectByName(name);
    REQUIRE(found != nullptr);
    REQUIRE(found->material() != nullptr);

    return *found->material();
}

}

TEST_CASE("the point a surface is linearized at is marked at (p, f(p)) and follows p with its tangent plane", "[optimization][surface][linearization]")
{
    const std::unique_ptr<keyed_surface> keyed = circle_surface();
    keyed->linearized_at(Eigen::Vector2d(0.5, -0.5));
    check_at(*keyed, Eigen::Vector2d(0.5, -0.5));

    keyed->linearized_at(Eigen::Vector2d(-1.2, 0.8));
    check_at(*keyed, Eigen::Vector2d(-1.2, 0.8));
}

TEST_CASE("the mark at p hides while p lies outside the ranges and returns with it", "[optimization][surface][linearization]")
{
    const std::unique_ptr<keyed_surface> keyed = circle_surface();
    keyed->linearized_at(Eigen::Vector2d(0.5, -0.5));
    CHECK(drawn_visible(keyed->scene(), mark));

    keyed->linearized_at(Eigen::Vector2d(3.0, 0.0));
    REQUIRE(published(*keyed)->linearization.has_value());
    REQUIRE(published(*keyed)->linearization->point == Eigen::Vector2d(3.0, 0.0));
    CHECK_FALSE(drawn_visible(keyed->scene(), mark));

    keyed->linearized_at(Eigen::Vector2d(0.5, -0.5));
    check_at(*keyed, Eigen::Vector2d(0.5, -0.5));
}

TEST_CASE("the mark at p is hidden with the Linearization switch off and where no point was linearized", "[optimization][surface][linearization]")
{
    const std::unique_ptr<keyed_surface> keyed = circle_surface();
    keyed->set(&scenario_view::linearization, true);
    REQUIRE_FALSE(published(*keyed)->linearization.has_value());
    CHECK_FALSE(drawn_visible(keyed->scene(), mark));

    keyed->linearized_at(Eigen::Vector2d(0.5, -0.5));
    CHECK(drawn_visible(keyed->scene(), mark));

    keyed->set(&scenario_view::linearization, false);
    CHECK_FALSE(drawn_visible(keyed->scene(), mark));
}

TEST_CASE("over a minimized surface the mark at p is drawn after the translucent field and untested against its depths", "[optimization][surface][linearization]")
{
    keyed_surface keyed(elliptic(), elliptic_minimum(problem_form::minimum), false);
    keyed.linearized_at(Eigen::Vector2d::Zero());
    threepp::Scene &scene = keyed.scene();
    REQUIRE(drawn_visible(scene, mark));
    CHECK(material_of(scene, "surface").transparent);

    const threepp::Material &drawn = material_of(scene, mark);
    CHECK(drawn.transparent);
    CHECK(drawn.opacity == 1.f);
    CHECK_FALSE(drawn.depthTest);
    CHECK_FALSE(drawn.depthWrite);
    CHECK(scene.getObjectByName(mark)->renderOrder > scene.getObjectByName("surface")->renderOrder);
}
