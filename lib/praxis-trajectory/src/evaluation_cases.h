#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_EVALUATION_CASES_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_EVALUATION_CASES_H

#include "praxis/trajectory/path.h"
#include "praxis/trajectory/types.h"
#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/time_scaling.h"

#include "praxis/evaluation/generation.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/types.h"

#include <span>
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

// Seconds.
inline constexpr double longest_duration = 4.0;

// Path parameter per second and per second squared. A trapezoidal profile over a unit travel reaches
// its cruise phase exactly where the speed bound squared stands under the acceleration bound, and
// the two ranges here straddle that ratio.
inline constexpr double slowest_speed_bound         = 0.4;
inline constexpr double fastest_speed_bound         = 3.0;
inline constexpr double gentlest_acceleration_bound = 0.5;
inline constexpr double harshest_acceleration_bound = 10.0;

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

// Drawn from one source in this order and no other: the acceleration, the cruise rate, the fraction
// of the duration the time is sampled at, then the instant the case is snapped to, the start, the
// ramp end, the coast end or the finish. The acceleration lies in [0.5, 10] per second squared and
// the cruise rate in [0.4, sqrt(a)) per second, so the case is a trapezoid choosing v and a; it
// carries the rate as its speed bound, the acceleration as its acceleration bound, and the duration
// (a + v^2)/(va) they give.
scaling_case drawn_velocity_acceleration_case(evaluation::case_source &drawn);

// Drawn as the case above, with the cruise rate drawn no lower than sqrt(0.05 a), so the duration it
// carries has vT no nearer 1 than 1.05.
scaling_case drawn_velocity_duration_case(evaluation::case_source &drawn);

// Drawn from one source in this order and no other: the acceleration, the duration, the fraction of
// the duration the time is sampled at, then the instant the case is snapped to, as above. The
// acceleration lies in [0.5, 10] per second squared and the duration in seconds over (2/sqrt(a), 4],
// so the case is a trapezoid choosing a and T; it carries the cruise rate those give as its speed
// bound.
scaling_case drawn_acceleration_duration_case(evaluation::case_source &drawn);

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
// meters per second and radians per second.
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
// traversed in no time is the neighborhood in which the duration rule and the fit through the run
// degenerate, and that is the neighborhood the spread names. The row index is drawn under both
// spreads, so the two consume the same values in the same order. Under `bulk` alone, a joint run of
// at least three rows and two degrees of freedom holds one degree of freedom of one row after the
// first exactly at the row before's; the three values deciding it are drawn under both spreads.

// Via points in seconds, radians and radians per second, starting at time 0.
struct via_point_case
{
    std::vector<via_point> via_points;
};

// Drawn from one source in this order and no other: the degree-of-freedom count, the via-point count
// in [2, 7], then per via point after the first a segment duration log-uniform over [0.05, 2], then
// per via point one position per degree of freedom and one velocity per degree of freedom in [-3, 3].
via_point_case drawn_via_point_case(evaluation::case_source &drawn);

// The uniform scalar every case here is built from. `angle_radians` is uniform over a full turn and
// is the only uniform scalar a source draws, so an affine map of it is uniform over the range that
// map reaches.
double over(evaluation::case_source &drawn, double from, double to);

// The share of cases moved onto an instant where the second derivative steps.
inline constexpr double snap_fraction = 0.25;

// The instant a case is moved onto, among those given, or `drawn` for the share of cases past the
// fraction given, which keep the time drawn for them.
snapped_instant snapped_among(evaluation::case_source &drawn, std::span<const snapped_instant> instants, double fraction);

// Seconds: the time an instant stands at in a profile of that duration and ramp.
double snapped_time(snapped_instant instant, double drawn_at, double duration, double ramp);

// One coordinate in radians per degree of freedom.
configuration drawn_configuration(evaluation::case_source &drawn, std::size_t coordinates);

// One screw implementation, one frame implementation, one set of paths and one set of time scalings
// serve every row in the module that needs them, so a row measures its own slot. Each is built once
// on first use and outlives every caller.
const rigid_motion::screw_ops &shared_screw();
const rigid_motion::frame_ops &shared_frames();
const path_ops &shared_paths();
const time_scaling_ops &shared_time_scalings();

}

#endif
