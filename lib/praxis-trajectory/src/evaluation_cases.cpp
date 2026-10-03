#include "evaluation_cases.h"

#include <span>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <utility>
#include <algorithm>

namespace praxis::trajectory {

namespace {

// Seconds.
constexpr double shortest_duration = 0.25;
constexpr double longest_duration  = 4.0;

// Path parameter per second and per second squared. A trapezoidal profile over a unit travel reaches
// its cruise phase exactly where the speed bound squared stands under the acceleration bound, and
// the two ranges here straddle that ratio.
constexpr double slowest_speed_bound         = 0.4;
constexpr double fastest_speed_bound         = 3.0;
constexpr double gentlest_acceleration_bound = 0.5;
constexpr double harshest_acceleration_bound = 10.0;

// The relative margin the manipulator's duration rule lengthens a natural trapezoid by.
constexpr double trapezoid_margin = 1.0e-5;

// The share of cases moved onto an instant where the second derivative steps.
constexpr double snap_fraction = 0.25;

constexpr std::array polynomial_kinks{snapped_instant::start, snapped_instant::finish};
constexpr std::array trapezoid_kinks{snapped_instant::start, snapped_instant::ramp_end, snapped_instant::coast_end, snapped_instant::finish};

// Lynch & Park, Modern Robotics, sec. 9.2.2.2, in the operand order of the manipulator's duration
// rule.
double natural_trapezoid_duration(double rate, double rate_change)
{
    const double peak = std::sqrt(rate_change);
    if(peak < rate)
        return peak / rate_change + peak / rate_change;

    const double ramp    = rate / rate_change;
    const double covered = 0.5 * rate_change * ramp * ramp;
    const double coast   = ((1.0 - covered) - covered) / rate;

    return ramp + (coast < 0.0 ? 0.0 : coast) + ramp;
}

// Under `near_singular` the draw lands a little below zero.
snapped_instant snapped_among(evaluation::case_source &drawn, std::span<const snapped_instant> instants)
{
    const double unit = over(drawn, 0.0, 1.0);
    if(!(unit < snap_fraction))
        return snapped_instant::drawn;

    const double last  = static_cast<double>(instants.size() - 1u);
    const double which = std::clamp(unit / snap_fraction * static_cast<double>(instants.size()), 0.0, last);

    return instants[static_cast<std::size_t>(which)];
}

double snapped_time(snapped_instant instant, double drawn_at, double duration, double ramp)
{
    switch(instant)
    {
        case snapped_instant::start:
            return 0.0;
        case snapped_instant::ramp_end:
            return ramp;
        case snapped_instant::coast_end:
            return duration - ramp;
        case snapped_instant::finish:
            return duration;
        case snapped_instant::drawn:
            break;
    }

    return drawn_at;
}

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
    const double duration         = over(drawn, shortest_duration, longest_duration);
    const double at               = over(drawn, 0.0, duration);
    const double speed            = over(drawn, slowest_speed_bound, fastest_speed_bound);
    const double acceleration     = over(drawn, gentlest_acceleration_bound, harshest_acceleration_bound);
    const snapped_instant snapped = snapped_among(drawn, polynomial_kinks);

    return scaling_case{duration, snapped_time(snapped, at, duration, 0.0), speed, acceleration, snapped};
}

scaling_case drawn_trapezoid_case(evaluation::case_source &drawn)
{
    const double duration_fraction = over(drawn, 0.0, 1.0);
    const double time_fraction     = over(drawn, 0.0, 1.0);
    const double speed             = over(drawn, slowest_speed_bound, fastest_speed_bound);
    const double acceleration      = over(drawn, gentlest_acceleration_bound, harshest_acceleration_bound);
    const double shortest          = trapezoid_duration_floor(speed, acceleration);
    const double duration          = shortest + (longest_duration - shortest) * duration_fraction;
    const snapped_instant snapped  = snapped_among(drawn, trapezoid_kinks);
    const double at                = snapped_time(snapped, duration * time_fraction, duration, trapezoid_ramp_duration(duration, acceleration));

    return scaling_case{duration, at, speed, acceleration, snapped};
}

double trapezoid_duration_floor(double speed_bound, double acceleration_bound)
{
    return natural_trapezoid_duration(speed_bound, acceleration_bound) * (1.0 + trapezoid_margin);
}

// Lynch & Park, Modern Robotics, sec. 9.2.2.2: t_a = v/a with v = (aT - sqrt(a) sqrt(aT^2 - 4))/2,
// evaluated as 2a/(aT + sqrt(a) sqrt(aT^2 - 4)).
double trapezoid_ramp_duration(double duration, double acceleration_bound)
{
    const double peak   = std::sqrt(acceleration_bound);
    const double cruise = 2.0 * acceleration_bound / (acceleration_bound * duration + peak * std::sqrt(acceleration_bound * duration * duration - 4.0));

    return cruise / acceleration_bound;
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
