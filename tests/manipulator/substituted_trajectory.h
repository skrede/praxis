#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_SUBSTITUTED_TRAJECTORY_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_SUBSTITUTED_TRAJECTORY_H

#include "praxis/trajectory/capabilities.h"

#include <cstddef>

// Each stub answers exactly what the reference answers and counts how often it was asked, so a
// composition binding one shows by the count alone whether the controller reached it.
namespace praxis::fixture {

inline std::size_t &velocity_acceleration_asks()
{
    static std::size_t asked = 0;

    return asked;
}

inline std::size_t &acceleration_duration_asks()
{
    static std::size_t asked = 0;

    return asked;
}

inline void reset_counts()
{
    velocity_acceleration_asks() = 0;
    acceleration_duration_asks() = 0;
}

inline expected<trajectory::scaling_sample, refusal> counted_velocity_acceleration(double t, double velocity, double acceleration)
{
    ++velocity_acceleration_asks();

    return trajectory::baseline().time_scaling.trapezoidal_from_velocity_acceleration(t, velocity, acceleration);
}

inline expected<trajectory::scaling_sample, refusal> counted_acceleration_duration(double t, double duration, double acceleration)
{
    ++acceleration_duration_asks();

    return trajectory::baseline().time_scaling.trapezoidal_from_acceleration_duration(t, duration, acceleration);
}

inline trajectory::capabilities only_the_velocity_acceleration_trapezoid()
{
    trajectory::capabilities shapes                            = trajectory::baseline();
    shapes.time_scaling.trapezoidal_from_velocity_acceleration = &counted_velocity_acceleration;

    return shapes;
}

inline trajectory::capabilities only_the_acceleration_duration_trapezoid()
{
    trajectory::capabilities shapes                            = trajectory::baseline();
    shapes.time_scaling.trapezoidal_from_acceleration_duration = &counted_acceleration_duration;

    return shapes;
}

}

#endif
