#include "drawn_graph.h"
#include "drawn_surface.h"
#include "keyed_surface.h"
#include "composed_curve.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/plot_window.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;

namespace {

// The part of the line through (x, g) within the range and the span: the width a tangent is drawn at.
interval held_part(double x, double g, double slope, const interval &range, const interval &span)
{
    if(slope == 0.0)
        return range;

    const double below = x + (span.lower - g) / slope;
    const double above = x + (span.upper - g) / slope;

    return interval{std::max(range.lower, std::min(below, above)), std::min(range.upper, std::max(below, above))};
}

// The series drawn before the tangent at x_k are those the graph fits its ordinates to.
interval fitted_span(const scene::plot_frame &frame)
{
    const auto first = std::ranges::find_if(frame.series, [](const scene::plot_series &series) { return series.name == "tangent at x_k"; });
    REQUIRE(first != frame.series.end());

    return drawn_span(frame, static_cast<std::size_t>(first - frame.series.begin()));
}

// The tangent at x spans [x - w / 2, x + w / 2], w the width of its part within the range and the span.
void check_centered(const scene::plot_frame &frame, const char *name, double x)
{
    const scene::plot_series *line = named(frame, name);
    REQUIRE(line != nullptr);
    REQUIRE(frame.abscissa_limits.has_value());
    check_on_line(*line, x, cubic_value(x), cubic_slope(x));

    const interval held = held_part(x, cubic_value(x), cubic_slope(x), interval{frame.abscissa_limits->lower, frame.abscissa_limits->upper}, fitted_span(frame));
    REQUIRE(held.lower < held.upper);
    const double half      = (held.upper - held.lower) / 2.0;
    const double tolerance = 1.0e-9 * std::max(1.0, std::abs(x));
    CHECK(std::abs(line->abscissa[0] - (x - half)) <= tolerance);
    CHECK(std::abs(line->abscissa[1] - (x + half)) <= tolerance);
}

double circle_value(double x, double y)
{
    return x * x + y * y - 1.0;
}

}

TEST_CASE("the tangent at x_k is centered on x_k at the width of its part within the range and the fitted ordinates", "[optimization][graph]")
{
    graph_stage stage(cubic(), baseline(), opening());
    REQUIRE(stage.ran->run.iterates.size() >= 3u);

    const scene::plot_frame frame = stage.at_step(1u);
    check_centered(frame, "tangent at x_k", stage.ran->run.iterates[1][0]);
}

TEST_CASE("the tangent at p is centered on p at the width of its part within the range and the fitted ordinates", "[optimization][graph][linearization]")
{
    graph_stage stage(cubic(), baseline(), opening());
    stage.taken.inputs->drawn->linearization = true;
    command(stage.taken.inputs->scenario, [](scenario_state &state) { state.linearize(Eigen::VectorXd::Constant(1, 1.5)); });

    const scene::plot_frame frame = stage.drained();
    REQUIRE(named(frame, "linearization point p") != nullptr);
    check_centered(frame, "tangent at p", 1.5);
}

TEST_CASE("the tangent plane at p is the rectangle of the ranges' widths centered on p, held to the ranges, at any height", "[optimization][surface][linearization]")
{
    const scenario_settings settings{"circle", {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, Eigen::Vector2d(1.0, 0.0), 70u, 0u, {}, {}};
    keyed_surface keyed(registered_surface("circle", &circle_value, [](double x, double y) { return Eigen::Vector2d(2.0 * x, 2.0 * y); }), settings, false);
    const Eigen::Vector2d p(0.5, -0.5);
    keyed.linearized_at(p);

    threepp::Scene &scene    = keyed.scene();
    const placement standing = placement_of(field_of(scene), *keyed.taken.inputs->seen.read());
    std::vector<Eigen::Vector2d> corners;
    for(const Eigen::Vector3d &drawn : drawn_points(scene, "tangent plane at p"))
    {
        const Eigen::Vector2d q = standing.over(drawn);
        CHECK(std::abs(drawn.z() - standing.scale * (circle_value(p.x(), p.y()) + (2.0 * p).dot(q - p))) <= 1.0e-4);
        if(std::ranges::none_of(corners, [&q](const Eigen::Vector2d &held) { return (held - q).norm() <= 1.0e-4; }))
            corners.push_back(q);
    }

    REQUIRE(corners.size() == 4u);
    for(const Eigen::Vector2d &corner : corners)
    {
        CHECK((std::abs(corner.x() + 1.5) <= 1.0e-4 || std::abs(corner.x() - 2.0) <= 1.0e-4));
        CHECK((std::abs(corner.y() + 2.0) <= 1.0e-4 || std::abs(corner.y() - 1.5) <= 1.0e-4));
    }
}
