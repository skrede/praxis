#include "drawn_surface.h"
#include "surface_step.h"
#include "keyed_surface.h"
#include "composed_curve.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/scenario_snapshot.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <cmath>
#include <vector>
#include <optional>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;

namespace {

const std::vector<interval> square{interval{-2.0, 2.0}, interval{-2.0, 2.0}};

double circle_value(double x, double y)
{
    return x * x + y * y - 1.0;
}

}

TEST_CASE("a zero gradient has a level tangent plane over the ranges, the one a vanishing gradient tends to, and no zero line or nearest point", "[optimization][surface][step]")
{
    for(const Eigen::Vector2d &x : {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(2.0, -2.0), Eigen::Vector2d(-1.5, 0.25)})
    {
        const std::optional<step_construction> level     = constructed_step(x, -1.0, Eigen::Vector2d::Zero(), square);
        const std::optional<step_construction> vanishing = constructed_step(x, -1.0, Eigen::Vector2d(1.0e-12, -1.0e-12), square);
        REQUIRE(level.has_value());
        REQUIRE(vanishing.has_value());
        CHECK_FALSE(level->zero_line.has_value());
        CHECK_FALSE(level->nearest.has_value());
        REQUIRE(level->patch.size() == vanishing->patch.size());
        for(const Eigen::Vector3d &corner : level->patch)
        {
            CHECK((corner.z() == -1.0 && std::abs(std::abs(corner.x()) - 2.0) <= 1.0e-12 && std::abs(std::abs(corner.y()) - 2.0) <= 1.0e-12));
            CHECK(std::ranges::any_of(vanishing->patch, [&corner](const Eigen::Vector3d &held) { return (held - corner).norm() <= 1.0e-10; }));
        }
    }
}

TEST_CASE("a seed at a zero gradient draws its level tangent plane over the ranges, and no zero line or nearest point", "[optimization][surface][step]")
{
    const scenario_settings settings{"circle", square, Eigen::Vector2d(0.0, 0.0), 70u, 0u, {}, {}};
    keyed_surface keyed(registered_surface("circle", &circle_value, [](double x, double y) { return Eigen::Vector2d(2.0 * x, 2.0 * y); }), settings, false);
    REQUIRE(keyed.taken.inputs->seen.read()->run.refused.has_value());

    threepp::Scene &scene                    = keyed.scene();
    const placement standing                 = placement_of(field_of(scene), *keyed.taken.inputs->seen.read());
    const std::vector<Eigen::Vector3d> drawn = drawn_points(scene, "tangent plane");
    CHECK(drawn_visible(scene, "tangent plane"));
    REQUIRE_FALSE(drawn.empty());
    for(const Eigen::Vector3d &corner : drawn)
    {
        const Eigen::Vector2d over = standing.over(corner);
        CHECK(std::abs(corner.z() - standing.scale * circle_value(0.0, 0.0)) <= 1.0e-6);
        CHECK((std::abs(std::abs(over.x()) - 2.0) <= 1.0e-5 && std::abs(std::abs(over.y()) - 2.0) <= 1.0e-5));
    }
    CHECK_FALSE(drawn_visible(scene, "zero line"));
    CHECK_FALSE(drawn_visible(scene, "nearest point"));
}
