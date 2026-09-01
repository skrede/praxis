#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_ANGLES_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_ANGLES_H

#include <numbers>

namespace praxis {

// Every extension signature carries angles in radians. Degrees exist only in the widget layer and
// are converted at that boundary.
inline constexpr double radians_per_degree = std::numbers::pi_v<double> / 180.0;
inline constexpr double degrees_per_radian = 180.0 / std::numbers::pi_v<double>;

constexpr double to_radians(double degrees)
{
    return degrees * radians_per_degree;
}

constexpr double to_degrees(double radians)
{
    return radians * degrees_per_radian;
}

}

#endif
