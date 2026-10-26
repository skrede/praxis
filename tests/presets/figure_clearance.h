#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_FIGURE_CLEARANCE_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_FIGURE_CLEARANCE_H

#include "standing_windows.h"

#include <Eigen/Core>

#include <span>
#include <limits>
#include <ranges>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::fixture {

struct disc
{
    Eigen::Vector2d center;
    double radius;
};

namespace detail {

inline double cross(const Eigen::Vector2d &from, const Eigen::Vector2d &to, const Eigen::Vector2d &at)
{
    const Eigen::Vector2d one = to - from, other = at - from;
    return one.x() * other.y() - one.y() * other.x();
}

// Andrew's monotone chain; each pass walks the points one way and keeps the turns of one sense.
inline std::vector<Eigen::Vector2d> hull_of(std::vector<Eigen::Vector2d> points)
{
    std::ranges::sort(points, [](const Eigen::Vector2d &one, const Eigen::Vector2d &other) { return one.x() < other.x() || (one.x() == other.x() && one.y() < other.y()); });
    std::vector<Eigen::Vector2d> hull;
    for(int pass = 0; pass < 2; ++pass)
    {
        const std::size_t start = hull.size();
        for(const Eigen::Vector2d &point : points)
        {
            while(hull.size() >= start + 2u && cross(hull[hull.size() - 2u], hull.back(), point) <= 0.0)
                hull.pop_back();
            hull.push_back(point);
        }
        hull.pop_back();
        std::ranges::reverse(points);
    }

    return hull;
}

inline double to_segment(const Eigen::Vector2d &at, const Eigen::Vector2d &from, const Eigen::Vector2d &to)
{
    const Eigen::Vector2d along = to - from;
    const double t              = along.squaredNorm() > 0.0 ? std::clamp((at - from).dot(along) / along.squaredNorm(), 0.0, 1.0) : 0.0;

    return (at - (from + t * along)).norm();
}

// Each vertex of either polygon against each edge of the other.
inline double apart(std::span<const Eigen::Vector2d> one, std::span<const Eigen::Vector2d> other)
{
    double least = std::numeric_limits<double>::infinity();
    for(int turn = 0; turn < 2; ++turn, std::swap(one, other))
        for(const Eigen::Vector2d &vertex : one)
            for(std::size_t edge = 0; edge < other.size(); ++edge)
                least = std::min(least, to_segment(vertex, other[edge], other[(edge + 1u) % other.size()]));

    return least;
}

// The widest gap between the two polygons' extents along any edge normal of either; positive only
// where the two are apart, since both are convex.
inline double widest_gap(std::span<const Eigen::Vector2d> one, std::span<const Eigen::Vector2d> other)
{
    double widest = -std::numeric_limits<double>::infinity();
    for(int turn = 0; turn < 2; ++turn, std::swap(one, other))
        for(std::size_t edge = 0; edge < one.size(); ++edge)
        {
            const Eigen::Vector2d along  = one[(edge + 1u) % one.size()] - one[edge];
            const Eigen::Vector2d normal = Eigen::Vector2d(-along.y(), along.x()).normalized();
            const auto extent            = [&normal](std::span<const Eigen::Vector2d> polygon)
            { return std::ranges::minmax(polygon | std::views::transform([&normal](const Eigen::Vector2d &vertex) { return vertex.dot(normal); })); };
            const auto first = extent(one), second = extent(other);
            widest = std::max({widest, second.min - first.max, first.min - second.max});
        }

    return widest;
}

inline std::vector<Eigen::Vector2d> corners_of(const standing_window &window)
{
    const double x = window.position.x, y = window.position.y, w = window.size.x, h = window.size.y;

    return {{x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}};
}

}

// Sharing an edge is not an overlap.
inline bool overlap(const standing_window &one, const standing_window &other)
{
    const auto crosses = [](float start, float extent, float other_start, float other_extent) { return start < other_start + other_extent && other_start < start + extent; };

    return crosses(one.position.x, one.size.x, other.position.x, other.size.x) && crosses(one.position.y, one.size.y, other.position.y, other.size.y);
}

// How far the disc stands outside the window, in px; negative where the two overlap.
inline double clearance(const disc &figure, const standing_window &window)
{
    const Eigen::Vector2d low(window.position.x, window.position.y);
    const Eigen::Vector2d high = low + Eigen::Vector2d(window.size.x, window.size.y);
    const Eigen::Vector2d nearest(std::clamp(figure.center.x(), low.x(), high.x()), std::clamp(figure.center.y(), low.y(), high.y()));
    if(nearest != figure.center)
        return (figure.center - nearest).norm() - figure.radius;

    return -std::min({figure.center.x() - low.x(), high.x() - figure.center.x(), figure.center.y() - low.y(), high.y() - figure.center.y()}) - figure.radius;
}

// How far the convex figure spanned by `points` stands outside the window, in px; at most 0 where the
// two overlap.
inline double clearance(std::vector<Eigen::Vector2d> points, const standing_window &window)
{
    const std::vector<Eigen::Vector2d> hull = detail::hull_of(std::move(points)), corners = detail::corners_of(window);
    const double gap = detail::widest_gap(hull, corners);

    return gap <= 0.0 ? gap : detail::apart(hull, corners);
}

}

#endif
