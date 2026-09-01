#include "praxis/manipulator/motion_commands.h"

#include <cmath>
#include <optional>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// A quintic time scaling peaks at 15/(8T) in the path parameter's first derivative and at
// 10/(sqrt(3) T^2) in its second: Lynch & Park, Modern Robotics, eq. (9.6) differentiated twice.
constexpr double velocity_peak     = 15.0 / 8.0;
constexpr double acceleration_peak = 10.0 / 1.7320508075688772;

// A cubic time scaling peaks at 3/(2T) and at 6/T^2 in the same two derivatives: Lynch & Park,
// Modern Robotics, eq. (9.5) differentiated twice.
constexpr double cubic_velocity_peak     = 3.0 / 2.0;
constexpr double cubic_acceleration_peak = 6.0;

// A motion whose bounds say nothing still has to take time; without a floor a trajectory would
// report a zero duration and end before its first sample.
constexpr double shortest_motion = 0.25;

// A motion of no extent leaves both derivatives of the path parameter unbounded; these stand in for
// the infinities the closed forms cannot divide by.
constexpr double unbounded_rate        = 1.0e3;
constexpr double unbounded_rate_change = 1.0e6;

double velocity_bounded(double distance, double limit, double peak)
{
    return limit > 0.0 ? peak * distance / limit : 0.0;
}

double acceleration_bounded(double distance, double limit, double peak)
{
    return limit > 0.0 ? std::sqrt(peak * distance / limit) : 0.0;
}

double peak_bounded_duration(const joint_limits &bounds, const joint_vector &start, const joint_vector &target, double first_peak, double second_peak)
{
    double span = shortest_motion;
    for(Eigen::Index i = 0; i < std::min(start.size(), target.size()); ++i)
    {
        const double distance = std::abs(target[i] - start[i]);
        if(i < bounds.velocity.size())
            span = std::max(span, velocity_bounded(distance, bounds.velocity[i], first_peak));
        if(i < bounds.acceleration.size())
            span = std::max(span, acceleration_bounded(distance, bounds.acceleration[i], second_peak));
    }

    return span;
}

// Lynch & Park, Modern Robotics, sec. 9.2.2: over a unit path a trapezoid coasts at its rate where
// the peak sqrt(a) reaches it and spans 1/v + v/a, and degenerates to the triangle spanning
// 2 sqrt(a)/a where it does not. Both branches are written in the ramp-and-coast form the bound
// profile computes its own duration in, down to the negative-coast guard and the order the three
// phases are added in: the profile is rescaled to what it is handed, rescaling only stretches, and a
// value one bit short of its own is refused.
double trapezoid_duration(const path_parameter_bounds &held_to)
{
    const double rate        = held_to.max_rate;
    const double rate_change = held_to.max_rate_change;
    if(!(rate > 0.0) || !(rate_change > 0.0))
        return shortest_motion;

    const double peak = std::sqrt(rate_change);
    if(peak < rate)
        return std::max(shortest_motion, peak / rate_change + peak / rate_change);

    const double ramp    = rate / rate_change;
    const double covered = 0.5 * rate_change * ramp * ramp;
    const double coast   = ((1.0 - covered) - covered) / rate;

    return std::max(shortest_motion, ramp + (coast < 0.0 ? 0.0 : coast) + ramp);
}

}

path_parameter_bounds derived_bounds(const joint_limits &bounds, const joint_vector &start, const joint_vector &target)
{
    path_parameter_bounds held{unbounded_rate, unbounded_rate_change};
    for(Eigen::Index i = 0; i < std::min(start.size(), target.size()); ++i)
    {
        const double distance = std::abs(target[i] - start[i]);
        if(!(distance > 0.0))
            continue;
        if(i < bounds.velocity.size() && bounds.velocity[i] > 0.0)
            held.max_rate = std::min(held.max_rate, bounds.velocity[i] / distance);
        if(i < bounds.acceleration.size() && bounds.acceleration[i] > 0.0)
            held.max_rate_change = std::min(held.max_rate_change, bounds.acceleration[i] / distance);
    }

    return held;
}

prepared_time_scaling::prepared_time_scaling(const trajectory::time_scaling_ops &injected_time_scaling, time_scaling_choice chosen)
        : m_chosen(chosen)
        , m_scaling(injected_time_scaling)
        , m_bounds(std::nullopt)
{
}

prepared_time_scaling::prepared_time_scaling(const trajectory::time_scaling_ops &injected_time_scaling, time_scaling_choice chosen, path_parameter_bounds held_to)
        : m_chosen(chosen)
        , m_scaling(injected_time_scaling)
        , m_bounds(held_to)
{
}

prepared_time_scaling::prepared_time_scaling(const prepared_time_scaling &scaled, path_parameter_bounds held_to)
        : m_chosen(scaled.m_chosen)
        , m_scaling(scaled.m_scaling)
        , m_bounds(held_to)
{
}

expected<trajectory::scaling_sample, refusal> prepared_time_scaling::sample(double t, double duration) const
{
    if(m_chosen == time_scaling_choice::cubic)
        return m_scaling.cubic(t, duration);
    if(m_chosen == time_scaling_choice::quintic)
        return m_scaling.quintic(t, duration);
    if(!m_bounds)
        return unexpected(refusal::unsupported_input);

    return m_scaling.trapezoidal(t, duration, m_bounds->max_rate, m_bounds->max_rate_change);
}

double prepared_time_scaling::duration(const joint_limits &bounds, const joint_vector &start, const joint_vector &target) const
{
    if(m_chosen == time_scaling_choice::cubic)
        return peak_bounded_duration(bounds, start, target, cubic_velocity_peak, cubic_acceleration_peak);
    if(m_chosen == time_scaling_choice::quintic)
        return peak_bounded_duration(bounds, start, target, velocity_peak, acceleration_peak);

    return trapezoid_duration(m_bounds ? *m_bounds : derived_bounds(bounds, start, target));
}

time_scaling_choice prepared_time_scaling::chosen() const
{
    return m_chosen;
}

std::optional<path_parameter_bounds> prepared_time_scaling::held_to() const
{
    return m_bounds;
}

}
