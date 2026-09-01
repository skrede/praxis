#include "evaluation_cases.h"

#include <cstddef>
#include <numbers>
#include <utility>

namespace praxis::trajectory {

namespace {

// Seconds.
constexpr double shortest_duration = 0.25;
constexpr double longest_duration  = 4.0;

// The fraction of the duration the sampled time reaches beyond each end of it.
constexpr double overhang = 0.25;

// Path parameter per second and per second squared. A trapezoidal profile over a unit travel reaches
// its cruise phase exactly where the speed bound squared stands under the acceleration bound, and
// the two ranges here straddle that ratio.
constexpr double slowest_speed_bound         = 0.4;
constexpr double fastest_speed_bound         = 3.0;
constexpr double gentlest_acceleration_bound = 0.5;
constexpr double harshest_acceleration_bound = 10.0;

}

double over(evaluation::case_source &drawn, double from, double to)
{
    const double unit = 0.5 * (1.0 + drawn.angle_radians() / std::numbers::pi_v<double>);

    return from + (to - from) * unit;
}

configuration drawn_configuration(evaluation::case_source &drawn, std::size_t coordinates)
{
    configuration values(static_cast<Eigen::Index>(coordinates));

    for(Eigen::Index axis = 0; axis < values.size(); ++axis)
        values[axis] = drawn.angle_radians();

    return values;
}

scaling_case drawn_scaling_case(evaluation::case_source &drawn)
{
    const double duration = over(drawn, shortest_duration, longest_duration);
    const double at       = over(drawn, -overhang * duration, (1.0 + overhang) * duration);

    return scaling_case{duration, at, over(drawn, slowest_speed_bound, fastest_speed_bound), over(drawn, gentlest_acceleration_bound, harshest_acceleration_bound)};
}

joint_path_case drawn_joint_path_case(evaluation::case_source &drawn)
{
    const std::size_t coordinates = drawn.axis_count();
    configuration start           = drawn_configuration(drawn, coordinates);
    configuration end             = drawn_configuration(drawn, coordinates);

    return joint_path_case{std::move(start), std::move(end), over(drawn, 0.0, 1.0)};
}

pose_path_case drawn_pose_path_case(evaluation::case_source &drawn)
{
    const transform start = drawn.transform_member();
    const transform end   = drawn.transform_member();

    return pose_path_case{start, end, over(drawn, 0.0, 1.0)};
}

}
