#include "praxis/trajectory/baseline/time_scaling.h"

#include <cmath>
#include <algorithm>

namespace praxis::trajectory {

namespace {

bool finite_and_positive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

expected<scaling_sample, refusal> refused()
{
    return unexpected(refusal::unsupported_input);
}

// Lynch & Park, Modern Robotics, sec. 9.2.2.2: T = (a + v^2)/(va), or a triangle peaking at sqrt(a)
// where v^2/a > 1.
double natural_duration(double rate, double rate_change)
{
    const double peak = std::sqrt(rate_change);
    if(peak < rate)
        return peak / rate_change + peak / rate_change;

    const double ramp    = rate / rate_change;
    const double covered = 0.5 * rate_change * ramp * ramp;
    const double coast   = ((1.0 - covered) - covered) / rate;

    return ramp + (coast < 0.0 ? 0.0 : coast) + ramp;
}

// Lynch & Park, Modern Robotics, sec. 9.2.2.2: v itself at the natural duration (the triangle's peak
// where v is never reached), and past it a held and v = (aT - sqrt(a) sqrt(aT^2 - 4))/2, evaluated
// as 2a/(aT + sqrt(a) sqrt(aT^2 - 4)).
double cruise_rate(double duration, double natural, double rate, double rate_change)
{
    const double peak = std::sqrt(rate_change);
    if(duration == natural)
        return peak < rate ? peak : rate;

    return 2.0 * rate_change / (rate_change * duration + peak * std::sqrt(std::max(0.0, rate_change * duration * duration - 4.0)));
}

// Lynch & Park, Modern Robotics, sec. 9.2.2.2, eq. (9.16)-(9.24), with t_a = v/a.
scaling_sample trapezoid_at(double t, double duration, double cruise, double rate_change)
{
    const double ramp = cruise / rate_change;
    if(t <= ramp)
        return {0.5 * rate_change * t * t, rate_change * t, rate_change};
    if(t <= duration - ramp)
        return {cruise * t - cruise * cruise / (2.0 * rate_change), cruise, 0.0};

    const double late = t - duration;

    return {(2.0 * rate_change * cruise * duration - 2.0 * cruise * cruise - rate_change * rate_change * late * late) / (2.0 * rate_change), rate_change * (duration - t), -rate_change};
}

}

// Lynch & Park, Modern Robotics, sec. 9.2.2.1, eq. (9.9) and (9.10); the end value is held outside
// [0, T].
expected<scaling_sample, refusal> cubic(double t, double duration)
{
    if(!finite_and_positive(duration))
        return refused();

    const double at = std::clamp(t, 0.0, duration);
    const double a2 = 3.0 / (duration * duration);
    const double a3 = -2.0 / (duration * duration * duration);

    return scaling_sample{a2 * at * at + a3 * at * at * at, 2.0 * a2 * at + 3.0 * a3 * at * at, 2.0 * a2 + 6.0 * a3 * at};
}

// Lynch & Park, Modern Robotics, sec. 9.2.2.1, Fig. 9.4: s = a3 t^3 + a4 t^4 + a5 t^5 with zero
// velocity and acceleration at both ends; the end value is held outside [0, T].
expected<scaling_sample, refusal> quintic(double t, double duration)
{
    if(!finite_and_positive(duration))
        return refused();

    const double at    = std::clamp(t, 0.0, duration);
    const double cubed = duration * duration * duration;
    const double a3    = 10.0 / cubed;
    const double a4    = -15.0 / (cubed * duration);
    const double a5    = 6.0 / (cubed * duration * duration);

    const double s   = a3 * at * at * at + a4 * at * at * at * at + a5 * at * at * at * at * at;
    const double ds  = 3.0 * a3 * at * at + 4.0 * a4 * at * at * at + 5.0 * a5 * at * at * at * at;
    const double dds = 6.0 * a3 * at + 12.0 * a4 * at * at + 20.0 * a5 * at * at * at;

    return scaling_sample{s, ds, dds};
}

// Lynch & Park, Modern Robotics, sec. 9.2.2.2. A duration below the natural one is refused, and the
// end value is held outside [0, T].
expected<scaling_sample, refusal> trapezoidal(double t, double duration, double max_velocity, double max_acceleration)
{
    if(!finite_and_positive(duration) || !finite_and_positive(max_velocity) || !finite_and_positive(max_acceleration))
        return refused();

    const double natural = natural_duration(max_velocity, max_acceleration);
    if(duration < natural)
        return refused();

    const double cruise = cruise_rate(duration, natural, max_velocity, max_acceleration);

    return trapezoid_at(std::clamp(t, 0.0, duration), duration, cruise, max_acceleration);
}

}
