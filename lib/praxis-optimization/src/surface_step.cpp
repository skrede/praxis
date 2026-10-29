#include "surface_step.h"
#include "scenario_defaults.h"

#include <span>
#include <array>
#include <cmath>
#include <vector>
#include <cstddef>
#include <optional>
#include <algorithm>

namespace praxis::optimization {

namespace {

// The points q with normal . q <= bound.
struct half_plane
{
    Eigen::Vector2d normal;
    double bound;
};

std::array<half_plane, 4> within(std::span<const interval> ranges)
{
    return {half_plane{Eigen::Vector2d(-1.0, 0.0), -ranges[0].lower}, half_plane{Eigen::Vector2d(1.0, 0.0), ranges[0].upper}, half_plane{Eigen::Vector2d(0.0, -1.0), -ranges[1].lower},
            half_plane{Eigen::Vector2d(0.0, 1.0), ranges[1].upper}};
}

// Sutherland & Hodgman (1974), one clipping plane: a convex polygon stays convex.
std::vector<Eigen::Vector2d> clipped(const std::vector<Eigen::Vector2d> &polygon, const half_plane &kept)
{
    std::vector<Eigen::Vector2d> inside;
    for(std::size_t at = 0; at < polygon.size(); ++at)
    {
        const Eigen::Vector2d &from = polygon[at];
        const Eigen::Vector2d &to   = polygon[(at + 1) % polygon.size()];
        const double beyond_from    = kept.normal.dot(from) - kept.bound;
        const double beyond_to      = kept.normal.dot(to) - kept.bound;
        if(beyond_from <= 0.0)
            inside.push_back(from);
        if((beyond_from < 0.0 && beyond_to > 0.0) || (beyond_from > 0.0 && beyond_to < 0.0))
            inside.push_back(from + beyond_from / (beyond_from - beyond_to) * (to - from));
    }

    return inside;
}

std::optional<std::array<Eigen::Vector2d, 2>> clipped(std::array<Eigen::Vector2d, 2> segment, const half_plane &kept)
{
    const double beyond_from = kept.normal.dot(segment[0]) - kept.bound;
    const double beyond_to   = kept.normal.dot(segment[1]) - kept.bound;
    if(beyond_from > 0.0 && beyond_to > 0.0)
        return std::nullopt;

    const Eigen::Vector2d cut = segment[0] + beyond_from / (beyond_from - beyond_to) * (segment[1] - segment[0]);
    if(beyond_from > 0.0)
        segment[0] = cut;
    else if(beyond_to > 0.0)
        segment[1] = cut;

    return segment;
}

std::vector<Eigen::Vector3d> lifted(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &x, double f, const Eigen::Vector2d &gradient)
{
    std::vector<Eigen::Vector3d> raised;
    for(const Eigen::Vector2d &q : polygon)
        raised.emplace_back(q.x(), q.y(), f + gradient.dot(q - x));

    return raised;
}

std::optional<std::array<Eigen::Vector2d, 2>> held(std::optional<std::array<Eigen::Vector2d, 2>> segment, std::span<const interval> ranges)
{
    for(const half_plane &kept : within(ranges))
        if(segment)
            segment = clipped(*segment, kept);

    return segment;
}

}

bool within_ranges(std::span<const interval> ranges, const Eigen::VectorXd &x)
{
    if(static_cast<std::size_t>(x.size()) != ranges.size())
        return false;

    for(std::size_t axis = 0; axis < ranges.size(); ++axis)
        if(!(x[static_cast<Eigen::Index>(axis)] >= ranges[axis].lower && x[static_cast<Eigen::Index>(axis)] <= ranges[axis].upper))
            return false;

    return true;
}

std::optional<step_construction> constructed_step(const Eigen::Vector2d &x, double f, const Eigen::Vector2d &gradient, std::span<const interval> ranges)
{
    const double squared = gradient.squaredNorm();
    if(!x.allFinite() || !std::isfinite(f) || !std::isfinite(squared) || squared == 0.0 || f == 0.0 || ranges.size() != 2u)
        return std::nullopt;

    const Eigen::Vector2d along   = gradient / std::sqrt(squared);
    const Eigen::Vector2d across  = Eigen::Vector2d(-along.y(), along.x());
    const Eigen::Vector2d nearest = x - gradient * (f / squared);
    const double reach            = std::hypot(ranges[0].upper - ranges[0].lower, ranges[1].upper - ranges[1].lower);
    // A square about a point within the ranges covers them whole once its half-side passes their
    // diagonal, so a larger one would only lose precision where it is held to them.
    const double half = std::min(tangent_patch * std::abs(f) / std::sqrt(squared), reach);

    std::vector<Eigen::Vector2d> square{x - half * (along + across), x + half * (along - across), x + half * (along + across), x - half * (along - across)};
    for(const half_plane &kept : within(ranges))
        square = clipped(square, kept);

    const std::array<Eigen::Vector2d, 2> crossing{nearest - half * across, nearest + half * across};
    const bool reached = (nearest - x).norm() <= half;

    return step_construction{lifted(square, x, f, gradient), reached ? held(crossing, ranges) : std::nullopt, nearest};
}

std::vector<Eigen::Vector3d> tangent_plane_over(const Eigen::Vector2d &p, double f, const Eigen::Vector2d &gradient, std::span<const interval> ranges)
{
    if(!p.allFinite() || !std::isfinite(f) || !gradient.allFinite() || ranges.size() != 2u)
        return {};

    Eigen::Vector2d lower;
    Eigen::Vector2d upper;
    for(Eigen::Index axis = 0; axis < 2; ++axis)
    {
        const interval &range = ranges[static_cast<std::size_t>(axis)];
        const double half     = (range.upper - range.lower) / 2.0;
        lower[axis]           = std::max(range.lower, p[axis] - half);
        upper[axis]           = std::min(range.upper, p[axis] + half);
        if(!(lower[axis] < upper[axis]))
            return {};
    }

    return lifted({lower, Eigen::Vector2d(upper.x(), lower.y()), upper, Eigen::Vector2d(lower.x(), upper.y())}, p, f, gradient);
}

}
