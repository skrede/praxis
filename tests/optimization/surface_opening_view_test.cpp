#include "drawn_surface.h"
#include "composed_curve.h"
#include "minimized_surface.h"
#include "scenario_defaults.h"

#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"

#include "praxis/scene/preset.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/math/Vector3.hpp>
#include <threepp/objects/Mesh.hpp>

#include <cmath>
#include <optional>

using namespace praxis;
using namespace praxis::fixture;

namespace {

void check_plane_opacity(minimized_surface &drawn, bool transparent, float opacity)
{
    for(const char *name : {"tangent plane at p", "central difference at p"})
    {
        const auto *plane = drawn.scene().getObjectByName<threepp::Mesh>(name);
        REQUIRE(plane != nullptr);
        CHECK(plane->material()->transparent == transparent);
        CHECK(plane->material()->opacity == opacity);
    }
}

void check_at(const threepp::Vector3 &seen, double x, double y, double z)
{
    CHECK(std::abs(static_cast<double>(seen.x) - x) <= 1.0e-6);
    CHECK(std::abs(static_cast<double>(seen.y) - y) <= 1.0e-6);
    CHECK(std::abs(static_cast<double>(seen.z) - z) <= 1.0e-6);
}

}

TEST_CASE("a surface composition opens at the view its elevation, azimuth and distance name, turning about the origin", "[optimization][surface][view]")
{
    const minimized_surface staged(problem_form::minimum);
    const std::optional<scene::camera_view> opening = staged.taken.composed->opening_view;
    REQUIRE(opening.has_value());

    const double level = opening_distance * std::cos(opening_elevation_radians);
    check_at(opening->position, level * std::sin(opening_azimuth_radians), opening_distance * std::sin(opening_elevation_radians), level * std::cos(opening_azimuth_radians));
    check_at(opening->target, 0.0, 0.0, 0.0);
}

TEST_CASE("a curve composition names no opening view", "[optimization][curve][view]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const opened taken = open_with(built.site, {cubic()}, baseline(), opening(), false);
    REQUIRE(settled(loop, taken) != nullptr);
    CHECK_FALSE(taken.composed->opening_view.has_value());
    taken.composed->tear_down();
}

TEST_CASE("a surface opens 50 degrees above the floor, unturned from +z, 1.8 m from the origin", "[optimization][surface][view]")
{
    const double degree = std::acos(-1.0) / 180.0;
    CHECK(std::abs(opening_elevation_radians - 50.0 * degree) <= 1.0e-9);
    CHECK(std::abs(opening_azimuth_radians - 0.0 * degree) <= 1.0e-9);
    CHECK(std::abs(opening_distance - 1.8) <= 1.0e-9);

    const minimized_surface staged(problem_form::minimum);
    const std::optional<scene::camera_view> opening = staged.taken.composed->opening_view;
    REQUIRE(opening.has_value());
    const double level = 1.8 * std::cos(50.0 * degree);
    check_at(opening->position, level * std::sin(0.0 * degree), 1.8 * std::sin(50.0 * degree), level * std::cos(0.0 * degree));
}

TEST_CASE("over a minimized surface the linearization planes are drawn at 0.6 opacity, over a root-form one opaque", "[optimization][surface][view]")
{
    minimized_surface minimized(problem_form::minimum);
    check_plane_opacity(minimized, true, 0.6f);
    CHECK(minimum_plane_opacity == 0.6);

    minimized_surface root(problem_form::root);
    check_plane_opacity(root, false, 1.f);
}
