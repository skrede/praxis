#include "straight_line.h"
#include "via_point_cubics.h"

#include "praxis/trajectory/baseline/trajectory.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::trajectory {

namespace {

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

expected<std::unique_ptr<trajectory_generator>, refusal> via_points(std::span<const configuration> waypoints, const configuration_limits &limits)
{
    if(!widths_agree(waypoints) || !all_finite(waypoints))
        return unexpected(refusal::unsupported_input);

    const std::vector<double> apportioned = knot_times(waypoints, limits.velocity);
    if(!rows_advance(apportioned))
        return unexpected(refusal::unsupported_input);

    via_point_run run               = fitted_run(waypoints, apportioned);
    const std::vector<double> times = bounded_times(run, limits.velocity);
    if(times.back() > apportioned.back())
        run = fitted_run(waypoints, times);

    return via_point_generator(std::move(run));
}

}

expected<std::unique_ptr<trajectory_generator>, refusal> joint_space_waypoints(std::span<const configuration> waypoints, const configuration &j0, const configuration_limits &limits)
{
    if(waypoints.size() == 1u)
        return straight_line(j0, waypoints.front(), limits);
    if(waypoints.size() == 2u)
        return straight_line(waypoints.front(), waypoints.back(), limits);
    if(waypoints.size() >= 3u)
        return via_points(waypoints, limits);

    spdlog::error("praxis: a run of waypoints joins at least one configuration and none were given");

    return unexpected(refusal::unsupported_input);
}

}
