#include "surface_step.h"
#include "curve_overlays.h"

#include "praxis/optimization/problem.h"

#include "praxis/scene/plot_window.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <algorithm>

using namespace praxis;
using namespace praxis::optimization;

namespace {

constexpr std::uint32_t tone = 0xf0e68cu;

const interval wide{-4.0, 4.0};

const std::array<interval, 2> square{interval{-2.0, 2.0}, interval{-2.0, 2.0}};

scene::plot_series drawn(const linearized &at, const interval &span)
{
    const std::optional<scene::plot_series> line = tangent("tangent", tone, at, wide, span);
    REQUIRE(line.has_value());
    REQUIRE(line->abscissa.size() == 2u);
    REQUIRE(line->ordinate.size() == 2u);
    CHECK_FALSE(line->fitted);
    for(std::size_t end = 0; end < 2u; ++end)
        CHECK(std::abs(line->ordinate[end] - (at.g + at.slope * (line->abscissa[end] - at.x))) <= 1.0e-12);

    return *line;
}

}

TEST_CASE("a tangent on the range's end keeps its width and reaches past the range by half of it", "[optimization][graph]")
{
    const scene::plot_series line = drawn(linearized{4.0, 0.0, 0.5}, interval{-10.0, 10.0});
    CHECK(line.abscissa == std::vector<double>{0.0, 8.0});
}

TEST_CASE("a tangent stopped by the fitted ordinates on one side extends as far on the other, past them", "[optimization][graph]")
{
    const scene::plot_series line = drawn(linearized{0.0, 0.9, 1.0}, interval{-1.0, 1.0});
    CHECK(std::abs(line.abscissa[0] + 1.0) <= 1.0e-12);
    CHECK(std::abs(line.abscissa[1] - 1.0) <= 1.0e-12);
    CHECK(line.ordinate[1] > 1.0);
}

TEST_CASE("a horizontal tangent at the range's middle spans the range, and one outside the fitted ordinates is not drawn", "[optimization][graph]")
{
    const scene::plot_series line = drawn(linearized{0.0, 0.5, 0.0}, interval{-1.0, 1.0});
    CHECK(line.abscissa == std::vector<double>{-4.0, 4.0});
    CHECK(line.ordinate == std::vector<double>{0.5, 0.5});
    CHECK_FALSE(tangent("tangent", tone, linearized{0.0, 2.0, 0.0}, wide, interval{-1.0, 1.0}).has_value());
}

TEST_CASE("the tangent plane at p covers the rectangle of the ranges' widths centered on p, cut at the ranges and at no height", "[optimization][surface]")
{
    const Eigen::Vector2d p(1.5, 0.5);
    const std::vector<Eigen::Vector3d> corners = tangent_plane_over(p, 0.0, Eigen::Vector2d(50.0, 0.0), square);
    REQUIRE(corners.size() == 4u);
    const std::array<Eigen::Vector2d, 4> expected{Eigen::Vector2d(-0.5, -1.5), Eigen::Vector2d(2.0, -1.5), Eigen::Vector2d(2.0, 2.0), Eigen::Vector2d(-0.5, 2.0)};
    for(std::size_t at = 0; at < corners.size(); ++at)
    {
        CHECK((corners[at].head<2>() - expected[at]).norm() == 0.0);
        CHECK(corners[at].z() == 50.0 * (expected[at].x() - p.x()));
    }

    const auto [low, high] = std::ranges::minmax(corners, {}, [](const Eigen::Vector3d &corner) { return corner.z(); });
    CHECK(low.z() == -100.0);
    CHECK(high.z() == 25.0);
}

TEST_CASE("the tangent plane at p is empty once p lies more than half a width outside the ranges", "[optimization][surface]")
{
    CHECK(tangent_plane_over(Eigen::Vector2d(4.5, 0.0), 1.0, Eigen::Vector2d(1.0, 1.0), square).empty());
    CHECK(tangent_plane_over(Eigen::Vector2d(0.0, -4.5), 1.0, Eigen::Vector2d(1.0, 1.0), square).empty());
}
