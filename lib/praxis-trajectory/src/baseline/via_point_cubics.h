#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_VIA_POINT_CUBICS_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_BASELINE_VIA_POINT_CUBICS_H

#include "praxis/trajectory/trajectory.h"

#include <Eigen/Core>

#include <span>
#include <memory>
#include <vector>

namespace praxis::trajectory {

// Row i holds degree of freedom i's a0, a1, a2, a3 in the time since the segment's first knot.
using segment_coefficient_block = Eigen::Matrix<double, Eigen::Dynamic, 4>;

// One cubic per segment per degree of freedom over one shared vector of knot times, so every degree
// of freedom stands at every via point at the same instant.
struct via_point_run
{
    std::vector<double> knots;
    std::vector<segment_coefficient_block> segments;
};

// The velocity at an interior via point from the slopes before and after it: zero where their signs
// differ, a zero slope having a sign of its own, and their mean otherwise. Biagiotti & Melchiorri,
// Trajectory Planning for Automatic Machines and Robots, sec. 2.1.4, eq. (2.3).
double via_velocity(double before, double after);

// a0, a1, a2, a3 of the cubic leaving `from` at `leaving` and reaching `to` at `arriving` after `span`.
// Lynch & Park, Modern Robotics, sec. 9.3, eq. (9.26)-(9.29).
Eigen::Vector4d segment_coefficients(double from, double to, double leaving, double arriving, double span);

// The run through `waypoints` at `knots`, with interior velocities by eq. (2.3) and at rest at both ends.
via_point_run fitted_run(std::span<const configuration> waypoints, const std::vector<double> &knots);

// The largest speed degree of freedom `dof` reaches over the run, taken per segment at both of its
// ends and where its acceleration crosses zero.
double peak_speed(const via_point_run &run, Eigen::Index dof);

// Samples the run, holding a time before the first knot or after the last to that knot. A time at an
// interior knot belongs to the segment it begins, and the last knot to the last segment.
std::unique_ptr<trajectory_generator> via_point_generator(via_point_run run);

}

#endif
