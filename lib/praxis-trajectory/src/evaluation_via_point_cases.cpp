#include "evaluation_cases.h"

#include <cmath>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::trajectory {

namespace {

// Seconds. The segment durations of one run are log-uniform over these.
constexpr double shortest_via_segment = 0.05;
constexpr double longest_via_segment  = 2.0;

// Radians per second, per degree of freedom, at every via point, the ends included.
constexpr double fastest_via_velocity = 3.0;

constexpr std::size_t fewest_via_points = 2u;
constexpr std::size_t most_via_points   = 7u;

std::vector<double> drawn_times(evaluation::case_source &drawn, std::size_t count)
{
    std::vector<double> times = {0.0};
    times.reserve(count);
    for(std::size_t k = 1u; k < count; ++k)
        times.push_back(times.back() + std::exp(over(drawn, std::log(shortest_via_segment), std::log(longest_via_segment))));

    return times;
}

configuration drawn_velocity(evaluation::case_source &drawn, std::size_t coordinates)
{
    configuration velocity(static_cast<Eigen::Index>(coordinates));
    for(Eigen::Index axis = 0; axis < velocity.size(); ++axis)
        velocity[axis] = over(drawn, -fastest_via_velocity, fastest_via_velocity);

    return velocity;
}

}

via_point_case drawn_via_point_case(evaluation::case_source &drawn)
{
    const std::size_t coordinates = drawn.axis_count();
    const auto drawn_count        = static_cast<std::size_t>(over(drawn, static_cast<double>(fewest_via_points), static_cast<double>(most_via_points + 1u)));
    const std::size_t count       = std::clamp(drawn_count, fewest_via_points, most_via_points);
    const std::vector<double> at  = drawn_times(drawn, count);

    via_point_case example;
    example.via_points.reserve(count);
    for(double time : at)
    {
        configuration position = drawn_configuration(drawn, coordinates);
        example.via_points.push_back(via_point{time, std::move(position), drawn_velocity(drawn, coordinates)});
    }

    return example;
}

}
