#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_SUBSTITUTED_TRAJECTORY_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_SUBSTITUTED_TRAJECTORY_H

#include "praxis/trajectory/capabilities.h"

#include <span>
#include <memory>
#include <vector>
#include <cstddef>

// Each stub answers exactly what the reference answers and counts how often it was asked, so a
// composition binding one shows by the count alone whether the controller reached it. The via-point
// stub also keeps the last via points it joined.
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

inline std::size_t &via_point_schedule_asks()
{
    static std::size_t asked = 0;

    return asked;
}

inline std::size_t &via_point_slot_asks()
{
    static std::size_t asked = 0;

    return asked;
}

inline std::vector<trajectory::via_point> &last_joined()
{
    static std::vector<trajectory::via_point> joined;

    return joined;
}

inline void reset_counts()
{
    velocity_acceleration_asks() = 0;
    acceleration_duration_asks() = 0;
    via_point_schedule_asks()    = 0;
    via_point_slot_asks()        = 0;
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

inline expected<std::vector<trajectory::via_point>, refusal> counted_via_point_schedule(std::span<const trajectory::configuration> positions, const trajectory::configuration &j0,
                                                                                        const trajectory::configuration_limits &limits)
{
    ++via_point_schedule_asks();

    return trajectory::baseline().trajectory.via_point_schedule(positions, j0, limits);
}

inline expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> counted_via_points(std::span<const trajectory::via_point> via_points)
{
    ++via_point_slot_asks();
    last_joined().assign(via_points.begin(), via_points.end());

    return trajectory::baseline().trajectory.joint_space_via_points(via_points);
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

inline trajectory::capabilities only_the_via_point_schedule()
{
    trajectory::capabilities shapes      = trajectory::baseline();
    shapes.trajectory.via_point_schedule = &counted_via_point_schedule;

    return shapes;
}

inline trajectory::capabilities only_the_via_point_slot()
{
    trajectory::capabilities shapes          = trajectory::baseline();
    shapes.trajectory.joint_space_via_points = &counted_via_points;

    return shapes;
}

}

#endif
