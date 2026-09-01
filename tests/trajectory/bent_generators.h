#ifndef HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_BENT_GENERATORS_H
#define HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_BENT_GENERATORS_H

#include "bent_trajectory.h"

#include "praxis/trajectory/baseline/trajectory.h"
#include "praxis/trajectory/baseline/pose_trajectory.h"

#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/pose_trajectory.h"

#include <span>
#include <array>
#include <memory>
#include <cstddef>
#include <utility>

namespace praxis::fixture {

inline trajectory::trajectory_sample displaced(const trajectory::trajectory_sample &point)
{
    trajectory::trajectory_sample moved = point;

    moved.position[0] += bent_by(bent::element_wise);

    return moved;
}

// The angular half of a twist is its first three components and the linear half its last three, so a
// run setting one entry and leaving the other at zero moves that half of a sample and no other.
inline trajectory::pose_sample displaced(const trajectory::pose_sample &point)
{
    trajectory::pose_sample moved = point;

    moved.position = displaced(point.position);
    moved.velocity.head<3>()[0] += bent_by(bent::twist_radians);
    moved.velocity.tail<3>()[0] += bent_by(bent::twist_metres);

    return moved;
}

// The reference's own generator, answering the samples it answers displaced. The duration is the
// reference's, so what a run of this reports differs only in what it says at a time.
template<typename Generator, typename Sample>
class displacing_generator : public Generator
{
public:
    explicit displacing_generator(std::unique_ptr<Generator> held)
            : m_held(std::move(held))
    {
    }

    expected<Sample, refusal> sample(double t) const override
    {
        const expected<Sample, refusal> read = m_held->sample(t);
        if(!read)
            return read;

        return displaced(*read);
    }

    double duration() const override
    {
        return m_held->duration();
    }

private:
    std::unique_ptr<Generator> m_held;
};

template<typename Generator, typename Sample, typename Answered>
expected<std::unique_ptr<Generator>, refusal> wrapping(Answered &&answered)
{
    if(!answered)
        return unexpected(answered.error());

    return std::unique_ptr<Generator>(std::make_unique<displacing_generator<Generator, Sample>>(std::move(*answered)));
}

inline expected<std::unique_ptr<trajectory::trajectory_generator>, refusal>
displaced_joint_space_waypoints(std::span<const trajectory::configuration> waypoints, const trajectory::configuration &j0, const trajectory::configuration_limits &limits)
{
    return wrapping<trajectory::trajectory_generator, trajectory::trajectory_sample>(trajectory::joint_space_waypoints(waypoints, j0, limits));
}

inline expected<std::unique_ptr<trajectory::pose_trajectory_generator>, refusal> displaced_decoupled_pose_waypoints(std::span<const transform> waypoints, const transform &seed,
                                                                                                                    double max_linear_speed, double max_angular_speed)
{
    return wrapping<trajectory::pose_trajectory_generator, trajectory::pose_sample>(trajectory::decoupled_pose_waypoints(waypoints, seed, max_linear_speed, max_angular_speed));
}

inline expected<std::unique_ptr<trajectory::pose_trajectory_generator>, refusal> displaced_screw_pose_waypoints(std::span<const transform> waypoints, const transform &seed,
                                                                                                                double max_linear_speed, double max_angular_speed)
{
    return wrapping<trajectory::pose_trajectory_generator, trajectory::pose_sample>(trajectory::screw_pose_waypoints(waypoints, seed, max_linear_speed, max_angular_speed));
}

inline void bend_decoupled_pose_waypoints(trajectory::capabilities &shapes)
{
    shapes.pose_trajectory.decoupled_pose_waypoints = &displaced_decoupled_pose_waypoints;
}

inline void bend_screw_pose_waypoints(trajectory::capabilities &shapes)
{
    shapes.pose_trajectory.screw_pose_waypoints = &displaced_screw_pose_waypoints;
}

inline void bend_joint_space_waypoints(trajectory::capabilities &shapes)
{
    shapes.trajectory.joint_space_waypoints = &displaced_joint_space_waypoints;
}

// The index is the order the report lists slots in: the three time scalings in enumerator order, the
// three paths, the two pose runs, then the configuration-space run. The function is total, so an
// index past the last slot answers the reference unchanged and no visible difference.
inline constexpr std::array<bend_applier, 9> bends{&bend_cubic,
                                                   &bend_quintic,
                                                   &bend_trapezoidal,
                                                   &bend_joint_straight_line,
                                                   &bend_screw,
                                                   &bend_decoupled,
                                                   &bend_decoupled_pose_waypoints,
                                                   &bend_screw_pose_waypoints,
                                                   &bend_joint_space_waypoints};

inline trajectory::capabilities bent_at(std::size_t index)
{
    trajectory::capabilities shapes = trajectory::baseline();

    if(index < bends.size())
        bends[index](shapes);

    return shapes;
}

inline trajectory::capabilities bent_everywhere()
{
    trajectory::capabilities shapes = trajectory::baseline();

    for(bend_applier bend : bends)
        bend(shapes);

    return shapes;
}

}

#endif
