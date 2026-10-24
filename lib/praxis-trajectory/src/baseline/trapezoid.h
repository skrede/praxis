#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_TRAPEZOID_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_TRAPEZOID_H

#include <cmath>
#include <algorithm>

namespace praxis::trajectory {

// Lynch & Park, Modern Robotics, sec. 9.2.2.2: T = (a + v^2)/(va), or a triangle peaking at sqrt(a)
// where v^2/a > 1.
inline double natural_trapezoid_duration(double rate, double rate_change)
{
    const double peak = std::sqrt(rate_change);
    if(peak < rate)
        return peak / rate_change + peak / rate_change;

    const double ramp    = rate / rate_change;
    const double covered = 0.5 * rate_change * ramp * ramp;
    const double coast   = ((1.0 - covered) - covered) / rate;

    return ramp + (coast < 0.0 ? 0.0 : coast) + ramp;
}

// Lynch & Park, Modern Robotics, sec. 9.2.2.2, choosing a and T: v = (aT - sqrt(a) sqrt(aT^2 - 4))/2,
// evaluated as 2a/(aT + sqrt(a) sqrt(aT^2 - 4)).
inline double trapezoid_cruise_rate(double duration, double rate_change)
{
    return 2.0 * rate_change / (rate_change * duration + std::sqrt(rate_change) * std::sqrt(std::max(0.0, rate_change * duration * duration - 4.0)));
}

}

#endif
