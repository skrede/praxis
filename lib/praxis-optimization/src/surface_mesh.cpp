#include "surface_mesh.h"
#include "scenario_answers.h"
#include "scenario_defaults.h"

#include <threepp/core/BufferAttribute.hpp>

#include <span>
#include <array>
#include <cmath>
#include <vector>
#include <cstddef>
#include <algorithm>

namespace praxis::optimization {

namespace {

double tallest(const std::vector<double> &heights)
{
    double most = 0.0;
    for(const double f : heights)
        if(std::isfinite(f))
            most = std::max(most, std::abs(f));

    return most;
}

// The slope along one grid direction between the nearest finite neighbors on either side, the
// sample itself standing in for a missing one.
double slope(const std::vector<Eigen::Vector3d> &placed, const std::vector<double> &heights, std::size_t at, std::size_t before, std::size_t after, int axis)
{
    const std::size_t from = std::isfinite(heights[before]) ? before : at;
    const std::size_t to   = std::isfinite(heights[after]) ? after : at;
    const double run       = placed[to][axis] - placed[from][axis];

    return run == 0.0 ? 0.0 : (placed[to].z() - placed[from].z()) / run;
}

void write_normals(std::vector<float> &into, const std::vector<Eigen::Vector3d> &placed, const std::vector<double> &heights, std::size_t n)
{
    for(std::size_t j = 0; j < n; ++j)
        for(std::size_t i = 0; i < n; ++i)
        {
            const std::size_t at     = j * n + i;
            const double along_x     = slope(placed, heights, at, at - (i > 0 ? 1u : 0u), at + (i + 1 < n ? 1u : 0u), 0);
            const double along_y     = slope(placed, heights, at, at - (j > 0 ? n : 0u), at + (j + 1 < n ? n : 0u), 1);
            const Eigen::Vector3d up = Eigen::Vector3d(-along_x, -along_y, 1.0).normalized();
            for(int axis = 0; axis < 3; ++axis)
                into[3u * at + static_cast<std::size_t>(axis)] = static_cast<float>(up[axis]);
        }
}

std::size_t write_index(std::vector<unsigned int> &into, const std::vector<double> &heights, std::size_t n)
{
    const auto finite = [&heights](unsigned int at) { return std::isfinite(heights[at]); };

    std::size_t used = 0;
    for(std::size_t j = 0; j + 1 < n; ++j)
        for(std::size_t i = 0; i + 1 < n; ++i)
        {
            const auto a = static_cast<unsigned int>(j * n + i);
            const auto c = static_cast<unsigned int>(a + n);
            for(const std::array<unsigned int, 3> &corners : {std::array<unsigned int, 3>{a, a + 1u, c + 1u}, std::array<unsigned int, 3>{a, c + 1u, c}})
                if(std::ranges::all_of(corners, finite))
                    for(const unsigned int corner : corners)
                        into[used++] = corner;
        }

    return used;
}

}

Eigen::Vector3d field_placement::at(double x, double y, double f) const
{
    return Eigen::Vector3d(across * (x - middle.x()), across * (y - middle.y()), vertical * f);
}

field_placement placement_of(std::span<const interval> ranges, const scenario_grid &grid)
{
    const double wider = std::max(ranges[0].upper - ranges[0].lower, ranges[1].upper - ranges[1].lower);
    const double most  = tallest(grid.heights);
    const Eigen::Vector2d middle((ranges[0].lower + ranges[0].upper) / 2.0, (ranges[1].lower + ranges[1].upper) / 2.0);

    return field_placement{middle, surface_extent / wider, most > 0.0 ? surface_height * surface_extent / most : 0.0};
}

std::shared_ptr<threepp::BufferGeometry> field_geometry(std::size_t count)
{
    const std::size_t cells = (count - 1) * (count - 1);

    auto geometry = threepp::BufferGeometry::create();
    geometry->setAttribute("position", threepp::FloatBufferAttribute::create(std::vector<float>(3u * count * count), 3));
    geometry->setAttribute("normal", threepp::FloatBufferAttribute::create(std::vector<float>(3u * count * count), 3));
    geometry->setIndex(std::vector<unsigned int>(6u * cells));
    geometry->setDrawRange(0, 0);

    return geometry;
}

void write_field(threepp::BufferGeometry &shaped, const scenario_grid &grid, std::span<const interval> ranges, const field_placement &placed)
{
    auto *positions     = shaped.getAttribute<float>("position");
    auto *normals       = shaped.getAttribute<float>("normal");
    const std::size_t n = grid.count;
    if(positions == nullptr || normals == nullptr || shaped.getIndex() == nullptr || positions->array().size() != 3u * n * n)
        return;

    std::vector<Eigen::Vector3d> standing(n * n);
    for(std::size_t at = 0; at < n * n; ++at)
    {
        const double f = grid.heights[at];
        standing[at]   = placed.at(evenly(ranges[0], at % n, n), evenly(ranges[1], at / n, n), std::isfinite(f) ? f : 0.0);
        for(int axis = 0; axis < 3; ++axis)
            positions->array()[3u * at + static_cast<std::size_t>(axis)] = static_cast<float>(standing[at][axis]);
    }
    write_normals(normals->array(), standing, grid.heights, n);

    shaped.setDrawRange(0, static_cast<int>(write_index(shaped.getIndex()->array(), grid.heights, n)));
    positions->needsUpdate();
    normals->needsUpdate();
    shaped.getIndex()->needsUpdate();
    shaped.computeBoundingSphere();
}

}
