#ifndef HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_BENT_TRAJECTORY_H
#define HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_BENT_TRAJECTORY_H

#include "praxis/trajectory/baseline/path.h"
#include "praxis/trajectory/baseline/time_scaling.h"

#include "praxis/trajectory/path.h"
#include "praxis/trajectory/types.h"
#include "praxis/trajectory/capabilities.h"
#include "praxis/trajectory/time_scaling.h"

#include "praxis/rigid_motion/types.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <algorithm>

// One displacement per quantity a trajectory row's residual measures, and one binding per slot
// answering the reference displaced in the quantity that slot's own row measures. A displacement is
// set through the array rather than compiled in because a slot is a plain function pointer and
// carries nothing of its own.
namespace praxis::fixture {

enum class bent : std::size_t
{
    element_wise,
    pose_radians,
    pose_metres,
    twist_radians,
    twist_metres,
    count,
};

inline std::array<double, static_cast<std::size_t>(bent::count)> bend_by{};

inline double bent_by(bent which)
{
    return bend_by[static_cast<std::size_t>(which)];
}

inline void bend_every_row_by(const std::array<double, static_cast<std::size_t>(bent::count)> &by)
{
    std::copy(by.begin(), by.end(), bend_by.begin());
}

inline trajectory::scaling_sample displaced(const trajectory::scaling_sample &point)
{
    return trajectory::scaling_sample{point.s + bent_by(bent::element_wise), point.ds, point.dds};
}

// Both halves at once: the angular half turns the rotation block about x and the linear half moves
// the origin along it, so a run setting one entry and leaving the other at zero moves that half
// alone.
inline transform displaced(const transform &pose)
{
    const rotation further(Eigen::AngleAxisd(bent_by(bent::pose_radians), Eigen::Vector3d::UnitX()).toRotationMatrix());
    transform moved = pose;

    moved.block<3, 3>(0, 0) = further * pose.block<3, 3>(0, 0);
    moved(0, 3) += bent_by(bent::pose_metres);

    return moved;
}

template<typename T, typename Displace>
expected<T, refusal> moved_or_refused(const expected<T, refusal> &read, Displace displaced_by)
{
    if(!read)
        return read;

    return displaced_by(read.value());
}

inline expected<trajectory::scaling_sample, refusal> displaced_cubic(double t, double duration)
{
    return moved_or_refused(trajectory::cubic(t, duration), [](const trajectory::scaling_sample &point) { return displaced(point); });
}

inline expected<trajectory::scaling_sample, refusal> displaced_quintic(double t, double duration)
{
    return moved_or_refused(trajectory::quintic(t, duration), [](const trajectory::scaling_sample &point) { return displaced(point); });
}

inline expected<trajectory::scaling_sample, refusal> displaced_trapezoidal(double t, double duration, double max_velocity, double max_acceleration)
{
    return moved_or_refused(trajectory::trapezoidal(t, duration, max_velocity, max_acceleration), [](const trajectory::scaling_sample &point) { return displaced(point); });
}

inline expected<trajectory::configuration, refusal> displaced_joint_straight_line(const trajectory::configuration &start, const trajectory::configuration &end, double s)
{
    return moved_or_refused(trajectory::joint_straight_line(start, end, s),
                            [](const trajectory::configuration &reached)
                            {
                                trajectory::configuration moved = reached;
                                moved[0] += bent_by(bent::element_wise);

                                return moved;
                            });
}

inline expected<transform, refusal> displaced_screw(const transform &start, const transform &end, double s)
{
    return moved_or_refused(trajectory::screw(start, end, s), [](const transform &reached) { return displaced(reached); });
}

inline expected<transform, refusal> displaced_decoupled(const transform &start, const transform &end, double s)
{
    return moved_or_refused(trajectory::decoupled(start, end, s), [](const transform &reached) { return displaced(reached); });
}

using bend_applier = void (*)(trajectory::capabilities &shapes);

inline void bend_cubic(trajectory::capabilities &shapes)
{
    shapes.time_scaling.cubic = &displaced_cubic;
}

inline void bend_quintic(trajectory::capabilities &shapes)
{
    shapes.time_scaling.quintic = &displaced_quintic;
}

inline void bend_trapezoidal(trajectory::capabilities &shapes)
{
    shapes.time_scaling.trapezoidal = &displaced_trapezoidal;
}

inline void bend_joint_straight_line(trajectory::capabilities &shapes)
{
    shapes.path.joint_straight_line = &displaced_joint_straight_line;
}

inline void bend_screw(trajectory::capabilities &shapes)
{
    shapes.path.screw = &displaced_screw;
}

inline void bend_decoupled(trajectory::capabilities &shapes)
{
    shapes.path.decoupled = &displaced_decoupled;
}

}

#endif
