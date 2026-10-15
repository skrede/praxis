#include "evaluation_cases.h"

#include <array>
#include <cmath>
#include <algorithm>

namespace praxis::trajectory {

namespace {

// The relative margin the manipulator's duration rule lengthens a natural trapezoid by.
constexpr double trapezoid_margin = 1.0e-5;

// The relative margin a drawn cruise rate keeps under sqrt(a), so v^2/a <= 1 survives rounding.
constexpr double book_domain_margin = 1.0e-13;

// The least v^2/a a case choosing v and T is drawn at.
constexpr double least_book_rate_ratio = 0.05;

// The relative margin a duration choosing a and T is drawn above 2/sqrt(a) by.
constexpr double book_duration_floor_margin = 1.0e-4;

// The share of cases a book option's row moves onto an instant where the second derivative steps.
constexpr double book_snap_fraction = 0.3;

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

// Lynch & Park, Modern Robotics, sec. 9.2.2.2: T = (a + v^2)/(va) and t_a = v/a.
scaling_case drawn_chosen_rate_case(evaluation::case_source &drawn, double least_rate_ratio)
{
    const double acceleration     = over(drawn, gentlest_acceleration_bound, harshest_acceleration_bound);
    const double slowest          = std::max(slowest_speed_bound, std::sqrt(least_rate_ratio * acceleration));
    const double velocity         = over(drawn, slowest, std::sqrt(acceleration) * (1.0 - book_domain_margin));
    const double time_fraction    = over(drawn, 0.0, 1.0);
    const snapped_instant snapped = snapped_among(drawn, trapezoid_kinks, book_snap_fraction);
    const double duration         = (acceleration + velocity * velocity) / (velocity * acceleration);
    const double at               = snapped_time(snapped, duration * time_fraction, duration, velocity / acceleration);

    return scaling_case{duration, at, velocity, acceleration, snapped};
}

}

scaling_case drawn_trapezoid_case(evaluation::case_source &drawn)
{
    const double duration_fraction = over(drawn, 0.0, 1.0);
    const double time_fraction     = over(drawn, 0.0, 1.0);
    const double speed             = over(drawn, slowest_speed_bound, fastest_speed_bound);
    const double acceleration      = over(drawn, gentlest_acceleration_bound, harshest_acceleration_bound);
    const double shortest          = trapezoid_duration_floor(speed, acceleration);
    const double duration          = shortest + (longest_duration - shortest) * duration_fraction;
    const snapped_instant snapped  = snapped_among(drawn, trapezoid_kinks, snap_fraction);
    const double at                = snapped_time(snapped, duration * time_fraction, duration, trapezoid_ramp_duration(duration, acceleration));

    return scaling_case{duration, at, speed, acceleration, snapped};
}

scaling_case drawn_velocity_acceleration_case(evaluation::case_source &drawn)
{
    return drawn_chosen_rate_case(drawn, 0.0);
}

scaling_case drawn_velocity_duration_case(evaluation::case_source &drawn)
{
    return drawn_chosen_rate_case(drawn, least_book_rate_ratio);
}

// Lynch & Park, Modern Robotics, sec. 9.2.2.2, choosing a and T: v = (aT - sqrt(a) sqrt(aT^2 - 4))/2,
// evaluated as 2a/(aT + sqrt(a) sqrt(aT^2 - 4)), and t_a = v/a.
scaling_case drawn_acceleration_duration_case(evaluation::case_source &drawn)
{
    const double acceleration     = over(drawn, gentlest_acceleration_bound, harshest_acceleration_bound);
    const double peak             = std::sqrt(acceleration);
    const double duration         = over(drawn, 2.0 / peak * (1.0 + book_duration_floor_margin), longest_duration);
    const double time_fraction    = over(drawn, 0.0, 1.0);
    const snapped_instant snapped = snapped_among(drawn, trapezoid_kinks, book_snap_fraction);
    const double velocity         = 2.0 * acceleration / (acceleration * duration + peak * std::sqrt(acceleration * duration * duration - 4.0));
    const double at               = snapped_time(snapped, duration * time_fraction, duration, velocity / acceleration);

    return scaling_case{duration, at, velocity, acceleration, snapped};
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

}
