#include "level_curve.h"
#include "scenario_answers.h"

#include <span>
#include <array>
#include <cmath>
#include <vector>
#include <cstddef>
#include <algorithm>

namespace praxis::optimization {

namespace {

struct cell
{
    std::array<Eigen::Vector2d, 4> corners;
    std::array<double, 4> above;
};

// The corners run counterclockwise from the lower left; each edge is interpolated from its corner
// nearer the grid's origin, the order its neighbor shares.
constexpr std::array<std::array<std::size_t, 2>, 4> edges{{{0u, 1u}, {1u, 2u}, {3u, 2u}, {0u, 3u}}};

Eigen::Vector2d crossing(const cell &at, const std::array<std::size_t, 2> &edge)
{
    const double from = at.above[edge[0]];
    const double to   = at.above[edge[1]];

    return at.corners[edge[0]] + from / (from - to) * (at.corners[edge[1]] - at.corners[edge[0]]);
}

void add_cell(const cell &at, std::vector<level_segment> &segments)
{
    std::vector<Eigen::Vector2d> crossed;
    for(const std::array<std::size_t, 2> &edge : edges)
        if((at.above[edge[0]] < 0.0) != (at.above[edge[1]] < 0.0))
            crossed.push_back(crossing(at, edge));

    if(crossed.size() == 2u)
        segments.push_back({crossed[0], crossed[1]});
    if(crossed.size() != 4u)
        return;

    const double center = (at.above[0] + at.above[1] + at.above[2] + at.above[3]) / 4.0;
    if((center < 0.0) == (at.above[0] < 0.0))
    {
        segments.push_back({crossed[0], crossed[1]});
        segments.push_back({crossed[2], crossed[3]});
    }
    else
    {
        segments.push_back({crossed[3], crossed[0]});
        segments.push_back({crossed[1], crossed[2]});
    }
}

}

std::vector<level_segment> level_curve(const scenario_grid &grid, std::span<const interval> ranges, double level)
{
    const std::size_t n = grid.count;
    std::vector<level_segment> segments;
    if(n < 2u || ranges.size() != 2u || grid.heights.size() != n * n)
        return segments;

    for(std::size_t j = 0; j + 1 < n; ++j)
        for(std::size_t i = 0; i + 1 < n; ++i)
        {
            const std::array<std::size_t, 4> columns{i, i + 1, i + 1, i};
            const std::array<std::size_t, 4> rows{j, j, j + 1, j + 1};
            cell at{};
            for(std::size_t c = 0; c < 4u; ++c)
            {
                at.corners[c] = Eigen::Vector2d(evenly(ranges[0], columns[c], n), evenly(ranges[1], rows[c], n));
                at.above[c]   = grid.heights[rows[c] * n + columns[c]] - level;
            }
            if(std::ranges::all_of(at.above, [](double above) { return std::isfinite(above); }))
                add_cell(at, segments);
        }

    return segments;
}

}
