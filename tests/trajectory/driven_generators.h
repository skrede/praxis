#ifndef HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_DRIVEN_GENERATORS_H
#define HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_DRIVEN_GENERATORS_H

#include "praxis/trajectory/baseline/trajectory.h"

#include "praxis/trajectory/types.h"
#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/pose_trajectory.h"

#include "praxis/rigid_motion/types.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <span>
#include <memory>
#include <cstddef>
#include <utility>

// Bindings that answer a prepared generator rather than a value, written so that a run can read back
// what the comparison did with the object it was handed.
namespace praxis::fixture {

inline std::size_t durations_read = 0;
inline std::size_t samples_taken  = 0;

inline void forget_what_was_driven()
{
    durations_read = 0;
    samples_taken  = 0;
}

// The reference's own generator under a duration multiplied by `by`, counting every call the
// comparison makes. At a factor of one it answers exactly what the reference answers.
class stretched_trajectory : public trajectory::trajectory_generator
{
public:
    stretched_trajectory(std::unique_ptr<trajectory_generator> held, double by)
            : m_by(by)
            , m_held(std::move(held))
    {
    }

    expected<trajectory::trajectory_sample, refusal> sample(double t) const override
    {
        ++samples_taken;

        return m_held->sample(t);
    }

    double duration() const override
    {
        ++durations_read;

        return m_by * m_held->duration();
    }

private:
    double m_by;
    std::unique_ptr<trajectory_generator> m_held;
};

template<typename Generator, typename Sample, refusal Reason>
class refusing_generator : public Generator
{
public:
    expected<Sample, refusal> sample(double) const override
    {
        return unexpected(Reason);
    }

    double duration() const override
    {
        return 0.0;
    }
};

inline expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> stretched_by(double by, std::span<const trajectory::via_point> via_points)
{
    expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> answered = trajectory::joint_space_via_points(via_points);
    if(!answered)
        return unexpected(answered.error());

    return std::unique_ptr<trajectory::trajectory_generator>(std::make_unique<stretched_trajectory>(std::move(*answered), by));
}

// The factor a run's duration is stretched by: far enough above one that no bound on two spans
// agreeing admits it.
inline constexpr double stretch = 1.5;

inline expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> counted_joint_space_via_points(std::span<const trajectory::via_point> via_points)
{
    return stretched_by(1.0, via_points);
}

inline expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> stretched_joint_space_via_points(std::span<const trajectory::via_point> via_points)
{
    return stretched_by(stretch, via_points);
}

template<typename Generator, typename Sample, refusal Reason>
expected<std::unique_ptr<Generator>, refusal> answering_no_sample()
{
    return std::unique_ptr<Generator>(std::make_unique<refusing_generator<Generator, Sample, Reason>>());
}

inline expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> declining_joint_space_via_points(std::span<const trajectory::via_point>)
{
    return answering_no_sample<trajectory::trajectory_generator, trajectory::trajectory_sample, refusal::degenerate>();
}

inline expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> declining_unimplemented_joint_space_via_points(std::span<const trajectory::via_point>)
{
    return answering_no_sample<trajectory::trajectory_generator, trajectory::trajectory_sample, refusal::not_implemented>();
}

inline expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> unbuilt_joint_space_via_points(std::span<const trajectory::via_point>)
{
    return unexpected(refusal::degenerate);
}

inline expected<std::unique_ptr<trajectory::pose_trajectory_generator>, refusal> declining_pose_waypoints(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &,
                                                                                                          const trajectory::path_ops &, const trajectory::time_scaling_ops &,
                                                                                                          std::span<const transform>, const transform &, double, double)
{
    return answering_no_sample<trajectory::pose_trajectory_generator, trajectory::pose_sample, refusal::degenerate>();
}

inline expected<std::unique_ptr<trajectory::pose_trajectory_generator>, refusal> declining_unimplemented_pose_waypoints(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &,
                                                                                                                        const trajectory::path_ops &,
                                                                                                                        const trajectory::time_scaling_ops &, std::span<const transform>,
                                                                                                                        const transform &, double, double)
{
    return answering_no_sample<trajectory::pose_trajectory_generator, trajectory::pose_sample, refusal::not_implemented>();
}

inline expected<std::unique_ptr<trajectory::pose_trajectory_generator>, refusal> unbuilt_pose_waypoints(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &,
                                                                                                        const trajectory::path_ops &, const trajectory::time_scaling_ops &,
                                                                                                        std::span<const transform>, const transform &, double, double)
{
    return unexpected(refusal::degenerate);
}

}

#endif
