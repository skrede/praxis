#include "drawn_surface.h"
#include "composed_curve.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/objects/Mesh.hpp>

#include <threepp/materials/interfaces.hpp>

#include <Eigen/Core>

#include <string>

using namespace praxis;
using namespace praxis::fixture;

namespace {

threepp::Color worn(threepp::Scene &scene, const std::string &name)
{
    auto *drawn = scene.getObjectByName<threepp::Mesh>(name);
    REQUIRE(drawn != nullptr);
    const auto *material = dynamic_cast<const threepp::MaterialWithColor *>(drawn->material().get());
    REQUIRE(material != nullptr);

    return material->color;
}

}

TEST_CASE("the surface's plane at p wears the linearization tone and the step's plane the tangent's", "[optimization][surface][tone]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const registered_surface circle("circle", [](double x, double y) { return x * x + y * y - 1.0; }, [](double x, double y) { return Eigen::Vector2d(2.0 * x, 2.0 * y); });
    opened taken = open_surface_with(built.site, {circle}, baseline(),
                                     scenario_settings{"circle", {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, Eigen::Vector2d(-2.0, -2.0), 70u, 0u, {}, {}}, {}, true);
    REQUIRE(settled(loop, taken) != nullptr);
    command(taken.inputs->scenario, [](scenario_state &state) { state.linearize(Eigen::Vector2d(0.0, 0.0)); });
    REQUIRE(loop.drain().has_value());

    CHECK(worn(*built.scene, "tangent plane at p") == threepp::Color(0xffffffu));
    CHECK(worn(*built.scene, "central difference at p") == threepp::Color(0x808000u));
    CHECK(worn(*built.scene, "tangent plane") == threepp::Color(0xf0e68cu));
    taken.composed->tear_down();
}
