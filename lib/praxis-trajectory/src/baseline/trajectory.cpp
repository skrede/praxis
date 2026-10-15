#include "via_point_cubics.h"

#include "praxis/trajectory/baseline/trajectory.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <vector>
#include <cstddef>
#include <algorithm>

namespace praxis::trajectory {

namespace {

// The time the slowest degree of freedom needs to cross its own span at its velocity bound, which is
// the duration rule every segment of a configuration-space motion is apportioned by.
double traversal_time(const configuration &from, const configuration &to, const configuration &velocity)
{
    double span = 0.0;
    for(Eigen::Index i = 0; i < std::min(from.size(), to.size()); ++i)
        if(i < velocity.size() && velocity[i] > 0.0)
            span = std::max(span, std::abs(to[i] - from[i]) / velocity[i]);

    return span;
}

std::vector<double> knot_times(std::span<const configuration> waypoints, const configuration &velocity)
{
    std::vector<double> times = {0.0};
    times.reserve(waypoints.size());

    for(std::size_t k = 1u; k < waypoints.size(); ++k)
        times.push_back(times.back() + traversal_time(waypoints[k - 1u], waypoints[k], velocity));

    return times;
}

bool widths_agree(std::span<const configuration> waypoints)
{
    for(std::size_t k = 1u; k < waypoints.size(); ++k)
        if(waypoints[k].size() != waypoints.front().size())
        {
            spdlog::error("praxis: a run of waypoints joins configurations of one size, row {} holds {} where the first holds {}, and the run stands as it was given", k,
                          waypoints[k].size(), waypoints.front().size());
            return false;
        }

    return true;
}

bool all_finite(std::span<const configuration> waypoints)
{
    for(std::size_t k = 0u; k < waypoints.size(); ++k)
        for(Eigen::Index i = 0; i < waypoints[k].size(); ++i)
            if(!std::isfinite(waypoints[k][i]))
            {
                spdlog::error("praxis: row {} holds a coordinate that is not a finite number at degree of freedom {}, and the run stands as it was given", k, i);
                return false;
            }

    return true;
}

bool rows_advance(const std::vector<double> &times)
{
    for(std::size_t k = 1u; k < times.size(); ++k)
        if(!(times[k] > times[k - 1u]))
        {
            spdlog::error("praxis: row {} repeats the configuration the run already stands at, so it traverses in no time, and the run stands as it was given", k);
            return false;
        }

    return true;
}

// Stretching every knot time by one factor divides every rate the fit reports by that same factor,
// so the run stretched by the largest rate it reaches reaches its bounds and does not cross them.
std::vector<double> bounded_times(const via_point_run &run, const configuration &velocity)
{
    double reached = 1.0;
    for(Eigen::Index i = 0; i < std::min(velocity.size(), run.segments.front().rows()); ++i)
        if(velocity[i] > 0.0)
            reached = std::max(reached, peak_speed(run, i) / velocity[i]);

    std::vector<double> stretched;
    stretched.reserve(run.knots.size());
    for(double knot : run.knots)
        stretched.push_back(knot * reached);

    return stretched;
}

bool schedule_advances(std::size_t given, const std::vector<double> &times)
{
    if(given > 2u)
        return rows_advance(times);
    if(times.back() > 0.0)
        return true;

    spdlog::error("praxis: a run of one or two configurations that moves no joint whose velocity bound is positive does not move, and no via points are chosen for it");
    return false;
}

}

expected<std::vector<via_point>, refusal> via_point_schedule(std::span<const configuration> positions, const configuration &j0, const configuration_limits &limits)
{
    if(positions.empty())
    {
        spdlog::error("praxis: via points are chosen for at least one configuration and none were given");
        return unexpected(refusal::unsupported_input);
    }

    std::vector<configuration> rows(positions.begin(), positions.end());
    if(rows.size() == 1u)
        rows.insert(rows.begin(), j0);
    if(!widths_agree(rows) || !all_finite(rows))
        return unexpected(refusal::unsupported_input);

    const std::vector<double> apportioned = knot_times(rows, limits.velocity);
    if(!schedule_advances(positions.size(), apportioned))
        return unexpected(refusal::unsupported_input);

    return fitted_via_points(rows, bounded_times(fitted_run(rows, apportioned), limits.velocity));
}

}
