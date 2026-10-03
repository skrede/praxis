#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_EVALUATION_CASES_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_EVALUATION_CASES_H

#include "praxis/trajectory/types.h"

#include "praxis/evaluation/generation.h"

#include "praxis/rigid_motion/types.h"

#include <vector>
#include <cstddef>
#include <cstdint>

namespace praxis::trajectory {

// The instant a case was moved onto, where a scaling's second derivative steps, or `drawn` where the
// case kept the time drawn for it.
enum class snapped_instant : std::uint8_t
{
    drawn,
    start,
    ramp_end,
    coast_end,
    finish
};

// The duration and the time it is sampled at are in seconds. A time scaling carries the path
// parameter from zero to one over that duration, so the two bounds a trapezoidal profile is built
// under are per second and per second squared of that same dimensionless parameter.
struct scaling_case
{
    double duration;
    double at;
    double speed_bound;
    double acceleration_bound;
    snapped_instant snapped;
};

struct joint_path_case
{
    configuration start;
    configuration end;
    double s;
};

struct pose_path_case
{
    transform start;
    transform end;
    double s;
};

// Drawn from one source in this order and no other: the duration, the time it is sampled at, the
// speed bound, the acceleration bound, then the instant the case is snapped to, the start or the
// finish. The duration lies in [0.25, 4] seconds and the time in [0, duration].
scaling_case drawn_scaling_case(evaluation::case_source &drawn);

// Drawn from one source in this order and no other: the fraction of the drawable durations, the
// fraction of the duration the time is sampled at, the speed bound, the acceleration bound, then the
// instant the case is snapped to, the start, the ramp end, the coast end or the finish. The duration
// lies in [trapezoid_duration_floor, 4] seconds and the time in [0, duration]. The bounds straddle
// the ratio at which a trapezoidal profile stops reaching a cruise phase, so both shapes of that
// profile are drawn.
scaling_case drawn_trapezoid_case(evaluation::case_source &drawn);

// Seconds, under a speed bound per second and an acceleration bound per second squared.
double trapezoid_duration_floor(double speed_bound, double acceleration_bound);

// Seconds a trapezoid of that duration spends reaching its cruise rate under an acceleration bound
// per second squared.
double trapezoid_ramp_duration(double duration, double acceleration_bound);

// Drawn from one source in this order and no other: the coordinate count, one coordinate in radians
// per degree of freedom for the start, one per degree of freedom for the end, then the path
// parameter.
joint_path_case drawn_joint_path_case(evaluation::case_source &drawn);

// Drawn from one source in this order and no other: the start pose, the end pose, then the path
// parameter. The two poses are drawn independently, so the motion carrying one onto the other turns
// about an axis through neither origin, which is what parts the two task-space paths.
pose_path_case drawn_pose_path_case(evaluation::case_source &drawn);

// A run of configurations reached from a starting one, and the per-joint bounds its duration is
// derived from. The waypoints are in radians and the velocity bound in radians per second.
struct joint_waypoint_case
{
    std::vector<configuration> waypoints;
    configuration seed;
    configuration_limits limits;
};

// A run of poses reached from a starting one, and the two bounds its duration is derived from, in
// metres per second and radians per second.
struct pose_waypoint_case
{
    std::vector<transform> waypoints;
    transform seed;
    double max_linear_speed;
    double max_angular_speed;
};

// Drawn from one source in this order and no other: the degree-of-freedom count, one coordinate per
// degree of freedom for the starting configuration, the row count, then per row a step length and
// one coordinate per degree of freedom for its direction, then one velocity bound and one
// acceleration bound per degree of freedom, then the row a run stands still at, then whether a joint
// is held still, the row it is held still at and the degree of freedom held. The step lengths are
// log-uniform, so the segment durations of one run span decades.
joint_waypoint_case drawn_joint_waypoint_case(evaluation::case_source &drawn);

// Drawn from one source in this order and no other: the starting pose, the row count, one pose per
// row, the linear speed bound, the angular speed bound, then the row a run stands still at. The
// poses are drawn independently, so the motion carrying one onto the next turns about an axis
// through neither origin.
pose_waypoint_case drawn_pose_waypoint_case(evaluation::case_source &drawn);

// Both runs above stand still at one row under `near_singular` and at none under `bulk`: a segment
// traversed in no time is the neighbourhood in which the duration rule and the fit through the run
// degenerate, and that is the neighbourhood the spread names. The row index is drawn under both
// spreads, so the two consume the same values in the same order. Under `bulk` alone, a joint run of
// at least three rows and two degrees of freedom holds one degree of freedom of one row after the
// first exactly at the row before's; the three values deciding it are drawn under both spreads.

// The uniform scalar every case here is built from. `angle_radians` is uniform over a full turn and
// is the only uniform scalar a source draws, so an affine map of it is uniform over the range that
// map reaches.
double over(evaluation::case_source &drawn, double from, double to);

// One coordinate in radians per degree of freedom.
configuration drawn_configuration(evaluation::case_source &drawn, std::size_t coordinates);

}

#endif
