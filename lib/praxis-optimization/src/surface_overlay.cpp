#include "surface_step.h"
#include "surface_figures.h"
#include "surface_overlay.h"

#include <span>
#include <array>
#include <cmath>
#include <limits>
#include <vector>
#include <cstddef>
#include <optional>
#include <algorithm>

namespace praxis::optimization {

namespace {

struct answered
{
    Eigen::Vector2d x;
    double f;
    Eigen::Vector2d gradient;
};

std::optional<answered> answered_at(const scenario_run &run, std::size_t k, std::span<const interval> ranges)
{
    if(k + 1 >= run.iterates.size() || k >= run.errors.size() || k >= run.jacobians.size() || !within_ranges(ranges, run.iterates[k]))
        return std::nullopt;

    const expected<Eigen::VectorXd, refusal> &e        = run.errors[k];
    const expected<Eigen::MatrixXd, refusal> &jacobian = run.jacobians[k];
    if(!e || e->size() != 1 || !jacobian || jacobian->rows() != 1 || jacobian->cols() != 2)
        return std::nullopt;

    return answered{run.iterates[k], -(*e)[0], jacobian->row(0).transpose()};
}

void draw_recorded(threepp::Object3D &recorded, const scenario_snapshot &shown, const field_placement &placed)
{
    const std::size_t next  = shown.step + 1u;
    const scenario_run &run = shown.run;
    if(next >= run.iterates.size() || next >= run.errors.size() || !run.errors[next] || run.errors[next]->size() != 1 || !within_ranges(shown.ranges, run.iterates[next]))
        return;

    const Eigen::VectorXd &x = run.iterates[next];
    const std::array<Eigen::Vector3d, 2> rising{Eigen::Vector3d(x[0], x[1], 0.0), Eigen::Vector3d(x[0], x[1], -(*run.errors[next])[0])};
    write_points(recorded, rising, placed);
}

}

void draw_step(const step_figures &drawn, const scenario_snapshot &shown, const field_placement &placed)
{
    for(threepp::Object3D *figure : {static_cast<threepp::Object3D *>(&drawn.patch), &drawn.zero_line, &drawn.nearest, &drawn.recorded})
        figure->visible = false;

    draw_recorded(drawn.recorded, shown, placed);
    const std::optional<answered> at             = answered_at(shown.run, shown.step, shown.ranges);
    const std::optional<step_construction> built = at ? constructed_step(at->x, at->f, at->gradient, shown.ranges) : std::nullopt;
    if(!built)
        return;

    write_polygon(drawn.patch, built->patch, placed);
    if(built->zero_line)
    {
        const std::array<Eigen::Vector3d, 2> line{Eigen::Vector3d((*built->zero_line)[0].x(), (*built->zero_line)[0].y(), 0.0),
                                                  Eigen::Vector3d((*built->zero_line)[1].x(), (*built->zero_line)[1].y(), 0.0)};
        write_points(drawn.zero_line, line, placed);
    }
    if(within_ranges(shown.ranges, built->nearest))
        place_point(drawn.nearest, Eigen::Vector3d(built->nearest.x(), built->nearest.y(), 0.0), placed);
}

void draw_planes(const point_figures &drawn, const scenario_snapshot &shown, const field_placement &placed)
{
    drawn.plane.visible   = false;
    drawn.central.visible = false;
    drawn.point.visible   = false;
    if(!shown.linearization)
        return;

    const scenario_linearization &at = *shown.linearization;
    if(at.point.size() != 2 || !at.error || at.error->size() != 1 || !at.jacobian || at.jacobian->rows() != 1 || at.jacobian->cols() != 2)
        return;

    const double f = -(*at.error)[0];
    write_polygon(drawn.plane, tangent_plane_over(at.point, f, at.jacobian->row(0).transpose(), shown.ranges), placed);
    if(within_ranges(shown.ranges, at.point))
        place_point(drawn.point, Eigen::Vector3d(at.point[0], at.point[1], f), placed);
    if(at.check && at.check->disagrees && at.check->central.rows() == 1 && at.check->central.cols() == 2)
        write_polygon(drawn.central, tangent_plane_over(at.point, f, at.check->central.row(0).transpose(), shown.ranges), placed);
}

std::vector<Eigen::Vector3d> box_outline(const std::optional<box> &bounds, std::span<const interval> ranges)
{
    if(!bounds || bounds->lower.size() != 2 || bounds->upper.size() != 2 || ranges.size() != 2u)
        return {};

    std::array<double, 4> held{};
    for(std::size_t axis = 0; axis < 2u; ++axis)
    {
        const auto i        = static_cast<Eigen::Index>(axis);
        held[2u * axis]     = std::clamp(bounds->lower[i], ranges[axis].lower, ranges[axis].upper);
        held[2u * axis + 1] = std::clamp(bounds->upper[i], ranges[axis].lower, ranges[axis].upper);
    }
    if(!(held[0] < held[1]) || !(held[2] < held[3]))
        return {};

    const std::array<Eigen::Vector3d, 4> corners{Eigen::Vector3d(held[0], held[2], 0.0), Eigen::Vector3d(held[1], held[2], 0.0), Eigen::Vector3d(held[1], held[3], 0.0),
                                                 Eigen::Vector3d(held[0], held[3], 0.0)};
    std::vector<Eigen::Vector3d> sides;
    for(std::size_t at = 0; at < corners.size(); ++at)
        sides.insert(sides.end(), {corners[at], corners[(at + 1) % corners.size()]});

    return sides;
}

std::vector<Eigen::Vector3d> ends_at(std::span<const level_segment> segments, double f)
{
    std::vector<Eigen::Vector3d> points;
    points.reserve(2u * segments.size());
    for(const level_segment &segment : segments)
        for(const Eigen::Vector2d &end : segment)
            points.emplace_back(end.x(), end.y(), f);

    return points;
}

interval height_span(const scenario_grid &grid)
{
    interval span{std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
    for(const double f : grid.heights)
        if(std::isfinite(f))
            span = interval{std::min(span.lower, f), std::max(span.upper, f)};

    return span.lower <= span.upper ? span : interval{0.0, 0.0};
}

}
