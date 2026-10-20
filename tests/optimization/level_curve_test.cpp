#include "level_curve.h"
#include "scenario_defaults.h"

#include "praxis/optimization/scenario_snapshot.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <map>
#include <array>
#include <cmath>
#include <limits>
#include <vector>
#include <cstddef>
#include <utility>
#include <numeric>
#include <functional>

using namespace praxis::optimization;

namespace {

const std::vector<interval> square{interval{-2.0, 2.0}, interval{-2.0, 2.0}};

// A segment end within 1e-4 of the range's width of the zero set.
constexpr double within = 1.0e-4 * 4.0;

double along(std::size_t at, std::size_t count)
{
    return -2.0 + 4.0 / static_cast<double>(count - 1) * static_cast<double>(at);
}

scenario_grid sampled(const std::function<double(double, double)> &f)
{
    scenario_grid grid{surface_samples, {}};
    for(std::size_t j = 0; j < surface_samples; ++j)
        for(std::size_t i = 0; i < surface_samples; ++i)
            grid.heights.push_back(f(along(i, surface_samples), along(j, surface_samples)));

    return grid;
}

double circle(double x, double y)
{
    return x * x + y * y - 1.0;
}

std::pair<double, double> keyed(const Eigen::Vector2d &end)
{
    return {end.x(), end.y()};
}

// Ends the segments share, joined into components; a closed curve has every end shared by two.
std::size_t components(const std::vector<std::array<Eigen::Vector2d, 2>> &segments, bool &closed)
{
    std::vector<std::size_t> parent(segments.size());
    std::iota(parent.begin(), parent.end(), std::size_t{0});
    const std::function<std::size_t(std::size_t)> root = [&parent, &root](std::size_t at) { return parent[at] == at ? at : parent[at] = root(parent[at]); };

    std::map<std::pair<double, double>, std::vector<std::size_t>> ends;
    for(std::size_t at = 0; at < segments.size(); ++at)
        for(const Eigen::Vector2d &end : segments[at])
            ends[keyed(end)].push_back(at);

    closed = true;
    for(const auto &[end, sharing] : ends)
    {
        closed = closed && sharing.size() == 2u;
        for(const std::size_t other : sharing)
            parent[root(other)] = root(sharing.front());
    }

    std::size_t counted = 0;
    for(std::size_t at = 0; at < segments.size(); ++at)
        counted += root(at) == at ? 1u : 0u;

    return counted;
}

}

TEST_CASE("the unit circle's zero-level curve lies on the circle and closes into one component", "[optimization][level]")
{
    const std::vector<std::array<Eigen::Vector2d, 2>> segments = level_curve(sampled(&circle), square, 0.0);
    REQUIRE_FALSE(segments.empty());

    for(const std::array<Eigen::Vector2d, 2> &segment : segments)
        for(const Eigen::Vector2d &end : segment)
            CHECK(std::abs(end.norm() - 1.0) <= within);

    bool closed = false;
    CHECK(components(segments, closed) == 1u);
    CHECK(closed);
}

TEST_CASE("a field that never reaches the level gives no segment", "[optimization][level]")
{
    CHECK(level_curve(sampled([](double x, double y) { return x * x + y * y + 1.0; }), square, 0.0).empty());
}

TEST_CASE("a level of one half traces the circle of radius sqrt(1.5)", "[optimization][level]")
{
    const std::vector<std::array<Eigen::Vector2d, 2>> segments = level_curve(sampled(&circle), square, 0.5);
    REQUIRE_FALSE(segments.empty());

    for(const std::array<Eigen::Vector2d, 2> &segment : segments)
        for(const Eigen::Vector2d &end : segment)
            CHECK(std::abs(end.norm() - std::sqrt(1.5)) <= within);

    bool closed = false;
    CHECK(components(segments, closed) == 1u);
    CHECK(closed);
}

TEST_CASE("a cell with a corner that was not answered contributes no segment", "[optimization][level]")
{
    const scenario_grid whole = sampled(&circle);
    scenario_grid holed       = whole;

    const std::size_t i = surface_samples * 3u / 4u;
    const std::size_t j = surface_samples / 2u;
    const double x      = along(i, surface_samples);
    const double y      = along(j, surface_samples);
    REQUIRE(std::abs(std::hypot(x, y) - 1.0) < 0.05);
    holed.heights[j * surface_samples + i] = std::numeric_limits<double>::quiet_NaN();

    const double spacing                                       = 4.0 / static_cast<double>(surface_samples - 1);
    const std::vector<std::array<Eigen::Vector2d, 2>> segments = level_curve(holed, square, 0.0);
    CHECK(segments.size() < level_curve(whole, square, 0.0).size());
    for(const std::array<Eigen::Vector2d, 2> &segment : segments)
    {
        const Eigen::Vector2d middle = (segment[0] + segment[1]) / 2.0;
        CHECK_FALSE((std::abs(middle.x() - x) < spacing && std::abs(middle.y() - y) < spacing));
    }
}

// The corners above the level stand on one diagonal, and the mean of the four is above it too, so
// the two cut off the corners below rather than joining them.
TEST_CASE("a saddle cell is split by the value at its center", "[optimization][level]")
{
    const scenario_grid cell{2u, {1.0, -1.0, -1.0, 2.0}};
    const std::vector<interval> unit{interval{0.0, 1.0}, interval{0.0, 1.0}};

    const std::vector<std::array<Eigen::Vector2d, 2>> segments = level_curve(cell, unit, 0.0);
    REQUIRE(segments.size() == 2u);
    for(const std::array<Eigen::Vector2d, 2> &segment : segments)
    {
        const Eigen::Vector2d middle = (segment[0] + segment[1]) / 2.0;
        CHECK((middle.x() - 0.5) * (middle.y() - 0.5) < 0.0);
    }
}
