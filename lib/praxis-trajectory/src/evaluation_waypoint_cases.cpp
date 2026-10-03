#include "evaluation_cases.h"

#include <cmath>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::trajectory {

namespace {

// Radians. The step a row is reached by is log-uniform over these, so the segment durations of one
// run span three decades.
constexpr double shortest_step = 1.0e-3;
constexpr double longest_step  = 1.0;

// Radians per second and per second squared, per degree of freedom.
constexpr double slowest_joint_speed         = 0.4;
constexpr double fastest_joint_speed         = 3.0;
constexpr double gentlest_joint_acceleration = 0.5;
constexpr double harshest_joint_acceleration = 10.0;

// Metres per second and radians per second. A pair of poses traverses in the longer of the time its
// distance needs at the first and the time its turn needs at the second.
constexpr double slowest_linear_speed  = 0.4;
constexpr double fastest_linear_speed  = 6.0;
constexpr double slowest_angular_speed = 0.4;
constexpr double fastest_angular_speed = 6.0;

// The share of runs under `bulk`, of at least three rows and two degrees of freedom, in which one
// joint is held still across one segment.
constexpr double held_fraction = 0.125;

double log_over(evaluation::case_source &drawn, double from, double to)
{
    return std::exp(over(drawn, std::log(from), std::log(to)));
}

configuration per_joint(evaluation::case_source &drawn, std::size_t coordinates, double from, double to)
{
    configuration bound(static_cast<Eigen::Index>(coordinates));

    for(Eigen::Index axis = 0; axis < bound.size(); ++axis)
        bound[axis] = over(drawn, from, to);

    return bound;
}

// The extremes the run itself reaches, so the bounds a run is drawn under admit the run.
std::pair<configuration, configuration> spanned_by(const configuration &seed, const std::vector<configuration> &waypoints)
{
    configuration lower = seed;
    configuration upper = seed;

    for(const configuration &row : waypoints)
    {
        lower = lower.cwiseMin(row);
        upper = upper.cwiseMax(row);
    }

    return {std::move(lower), std::move(upper)};
}

template<typename Row>
void standing_still(evaluation::case_source &drawn, const Row &seed, std::vector<Row> &waypoints)
{
    const auto at = static_cast<std::size_t>(over(drawn, 0.0, static_cast<double>(waypoints.size()))) % waypoints.size();

    if(drawn.drawn_from() != evaluation::spread::near_singular)
        return;

    waypoints[at] = at == 0u ? seed : waypoints[at - 1u];
}

// Held to [from, count) at both ends: under `near_singular` a draw lands a little past either end.
std::size_t index_below(evaluation::case_source &drawn, std::size_t from, std::size_t count)
{
    const double at = std::max(static_cast<double>(from), over(drawn, static_cast<double>(from), static_cast<double>(count)));

    return std::min(static_cast<std::size_t>(at), count - 1u);
}

void holding_a_joint(evaluation::case_source &drawn, std::vector<configuration> &waypoints)
{
    const double share      = over(drawn, 0.0, 1.0);
    const std::size_t rows  = waypoints.size();
    const std::size_t width = static_cast<std::size_t>(waypoints.front().size());
    const std::size_t row   = index_below(drawn, 1u, rows);
    const auto axis         = static_cast<Eigen::Index>(index_below(drawn, 0u, width));

    if(drawn.drawn_from() != evaluation::spread::bulk || !(share < held_fraction) || rows < 3u || width < 2u)
        return;

    waypoints[row][axis] = waypoints[row - 1u][axis];
}

std::vector<configuration> reached_from(evaluation::case_source &drawn, const configuration &seed, std::size_t rows, std::size_t coordinates)
{
    std::vector<configuration> waypoints;
    waypoints.reserve(rows);

    for(std::size_t row = 0; row < rows; ++row)
    {
        const double reach = log_over(drawn, shortest_step, longest_step);

        waypoints.push_back((waypoints.empty() ? seed : waypoints.back()) + reach * drawn_configuration(drawn, coordinates));
    }

    return waypoints;
}

}

joint_waypoint_case drawn_joint_waypoint_case(evaluation::case_source &drawn)
{
    const std::size_t coordinates      = drawn.axis_count();
    configuration seed                 = drawn_configuration(drawn, coordinates);
    const std::size_t rows             = drawn.axis_count();
    std::vector<configuration> reached = reached_from(drawn, seed, rows, coordinates);

    configuration velocity     = per_joint(drawn, coordinates, slowest_joint_speed, fastest_joint_speed);
    configuration acceleration = per_joint(drawn, coordinates, gentlest_joint_acceleration, harshest_joint_acceleration);

    standing_still(drawn, seed, reached);
    holding_a_joint(drawn, reached);

    std::pair<configuration, configuration> held = spanned_by(seed, reached);
    configuration_limits limits{std::move(velocity), std::move(acceleration), std::move(held.first), std::move(held.second)};

    return joint_waypoint_case{std::move(reached), std::move(seed), std::move(limits)};
}

pose_waypoint_case drawn_pose_waypoint_case(evaluation::case_source &drawn)
{
    const transform seed   = drawn.transform_member();
    const std::size_t rows = drawn.axis_count();

    std::vector<transform> reached;
    reached.reserve(rows);
    for(std::size_t row = 0; row < rows; ++row)
        reached.push_back(drawn.transform_member());

    const double linear  = over(drawn, slowest_linear_speed, fastest_linear_speed);
    const double angular = over(drawn, slowest_angular_speed, fastest_angular_speed);

    standing_still(drawn, seed, reached);

    return pose_waypoint_case{std::move(reached), seed, linear, angular};
}

}
