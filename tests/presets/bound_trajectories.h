#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_BOUND_TRAJECTORIES_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_BOUND_TRAJECTORIES_H

#include "book_via_slots.h"
#include "book_trapezoid_options.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/trajectory/baseline/path.h"
#include "praxis/trajectory/baseline/trajectory.h"
#include "praxis/trajectory/baseline/time_scaling.h"
#include "praxis/trajectory/baseline/pose_trajectory.h"

#include "praxis/scene/plot_window.h"

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <vector>
#include <cstddef>
#include <algorithm>

namespace praxis::fixture {

using drawn_lines = std::vector<std::vector<Eigen::Vector3d>>;

// Bound one member at a time onto the inert aggregate, so a slot this list leaves out arrives inert.
inline trajectory::capabilities every_slot_the_reference()
{
    trajectory::capabilities bound;
    bound.time_scaling.cubic                                  = &trajectory::cubic;
    bound.time_scaling.quintic                                = &trajectory::quintic;
    bound.time_scaling.trapezoidal_from_velocity_acceleration = &trajectory::trapezoidal_from_velocity_acceleration;
    bound.time_scaling.trapezoidal_from_velocity_duration     = &trajectory::trapezoidal_from_velocity_duration;
    bound.time_scaling.trapezoidal_from_acceleration_duration = &trajectory::trapezoidal_from_acceleration_duration;
    bound.time_scaling.trapezoidal_within_bounds              = &trajectory::trapezoidal_within_bounds;
    bound.path.joint_straight_line                            = &trajectory::joint_straight_line;
    bound.path.screw                                          = &trajectory::screw;
    bound.path.decoupled                                      = &trajectory::decoupled;
    bound.pose_trajectory.decoupled_pose_waypoints            = &trajectory::decoupled_pose_waypoints;
    bound.pose_trajectory.screw_pose_waypoints                = &trajectory::screw_pose_waypoints;
    bound.trajectory.joint_space_via_points                   = &trajectory::joint_space_via_points;
    bound.trajectory.via_point_schedule                       = &trajectory::via_point_schedule;

    return bound;
}

static_assert(tests::option_shape_at(0).check == tests::domain_check::none);
static_assert(tests::via_slot_forms[0].spelling == tests::coefficient_spelling::printed);

// Lynch & Park, Modern Robotics, sec. 9.2.2.2 and eq. (9.26)-(9.29) as printed and unchecked, with the
// trapezoid within bounds and the step choosing the via points left to the reference.
inline trajectory::capabilities literal_book_slots()
{
    trajectory::capabilities bound                            = every_slot_the_reference();
    bound.time_scaling.trapezoidal_from_velocity_acceleration = &tests::book_velocity_acceleration_option<0>;
    bound.time_scaling.trapezoidal_from_velocity_duration     = &tests::book_velocity_duration_option<0>;
    bound.time_scaling.trapezoidal_from_acceleration_duration = &tests::book_acceleration_duration_option<0>;
    bound.trajectory.joint_space_via_points                   = &tests::literal_via_slot<0>;

    return bound;
}

// The realized d2s/dt2 at a run's resting end divides what rounding leaves of a vanished velocity by
// its own norm, so the last sample of that frame is no value two equal runs share and is left out.
inline scene::plot_reading short_of_the_rest(scene::plot_reading reading)
{
    if(reading.frames.empty())
        return reading;

    for(scene::plot_series &curve : reading.frames.back().series)
        if(!curve.ordinate.empty())
        {
            curve.abscissa.pop_back();
            curve.ordinate.pop_back();
        }

    return reading;
}

// The furthest any point of one set of lines stands from its counterpart in the other, and infinity
// where the two do not carry the same points.
inline double furthest_apart(const drawn_lines &first, const drawn_lines &second)
{
    if(first.size() != second.size())
        return std::numeric_limits<double>::infinity();

    double furthest = 0.0;
    for(std::size_t line = 0; line < first.size(); ++line)
    {
        if(first[line].size() != second[line].size())
            return std::numeric_limits<double>::infinity();
        for(std::size_t at = 0; at < first[line].size(); ++at)
            furthest = std::max(furthest, (first[line][at] - second[line][at]).norm());
    }

    return furthest;
}

inline double furthest_apart(const std::vector<double> &first, const std::vector<double> &second)
{
    if(first.size() != second.size())
        return std::numeric_limits<double>::infinity();

    double furthest = 0.0;
    for(std::size_t at = 0; at < first.size(); ++at)
    {
        const double apart = first[at] == second[at] ? 0.0 : std::abs(first[at] - second[at]);
        if(std::isnan(apart))
            return std::numeric_limits<double>::infinity();
        furthest = std::max(furthest, apart);
    }

    return furthest;
}

inline double furthest_apart(const scene::plot_reading &first, const scene::plot_reading &second)
{
    if(first.message != second.message || first.frames.size() != second.frames.size())
        return std::numeric_limits<double>::infinity();

    double furthest = 0.0;
    for(std::size_t frame = 0; frame < first.frames.size(); ++frame)
    {
        const std::vector<scene::plot_series> &one = first.frames[frame].series;
        const std::vector<scene::plot_series> &two = second.frames[frame].series;
        if(one.size() != two.size())
            return std::numeric_limits<double>::infinity();
        for(std::size_t curve = 0; curve < one.size(); ++curve)
            furthest = std::max({furthest, furthest_apart(one[curve].abscissa, two[curve].abscissa), furthest_apart(one[curve].ordinate, two[curve].ordinate)});
    }

    return furthest;
}

}

#endif
