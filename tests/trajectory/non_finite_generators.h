#ifndef HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_NON_FINITE_GENERATORS_H
#define HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_NON_FINITE_GENERATORS_H

#include "praxis/trajectory/baseline/trajectory.h"
#include "praxis/trajectory/baseline/pose_trajectory.h"

#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/pose_trajectory.h"

#include <span>
#include <limits>
#include <memory>
#include <utility>

namespace praxis::fixture {

using placement = bool (*)(double t, double duration);

inline bool inside(double t, double duration)
{
    return t > 0.0 && t < duration;
}

inline bool at_the_start(double t, double)
{
    return t <= 0.0;
}

template<auto Member, Eigen::Index... At>
void made_not_finite(auto &point)
{
    (point.*Member)(At...) = std::numeric_limits<double>::quiet_NaN();
}

// The reference's own generator, answering its samples with one value made not finite where the
// placement holds.
template<typename Generator, typename Sample, void (*Spoil)(Sample &), placement Where>
class not_finite_generator : public Generator
{
public:
    explicit not_finite_generator(std::unique_ptr<Generator> held)
            : m_held(std::move(held))
    {
    }

    expected<Sample, refusal> sample(double t) const override
    {
        expected<Sample, refusal> read = m_held->sample(t);
        if(read && Where(t, m_held->duration()))
            Spoil(*read);

        return read;
    }

    double duration() const override
    {
        return m_held->duration();
    }

private:
    std::unique_ptr<Generator> m_held;
};

template<typename Generator, typename Sample, void (*Spoil)(Sample &), placement Where>
expected<std::unique_ptr<Generator>, refusal> spoiling(expected<std::unique_ptr<Generator>, refusal> answered)
{
    if(!answered || *answered == nullptr)
        return answered;

    return std::unique_ptr<Generator>(std::make_unique<not_finite_generator<Generator, Sample, Spoil, Where>>(std::move(*answered)));
}

template<void (*Spoil)(trajectory::trajectory_sample &), placement Where>
expected<std::unique_ptr<trajectory::trajectory_generator>, refusal>
not_finite_joint_space_waypoints(std::span<const trajectory::configuration> waypoints, const trajectory::configuration &j0, const trajectory::configuration_limits &limits)
{
    return spoiling<trajectory::trajectory_generator, trajectory::trajectory_sample, Spoil, Where>(trajectory::joint_space_waypoints(waypoints, j0, limits));
}

template<void (*Spoil)(trajectory::pose_sample &)>
expected<std::unique_ptr<trajectory::pose_trajectory_generator>, refusal> not_finite_decoupled_pose_waypoints(std::span<const transform> waypoints, const transform &seed,
                                                                                                              double max_linear_speed, double max_angular_speed)
{
    return spoiling<trajectory::pose_trajectory_generator, trajectory::pose_sample, Spoil, &inside>(
            trajectory::decoupled_pose_waypoints(waypoints, seed, max_linear_speed, max_angular_speed));
}

template<void (*Spoil)(trajectory::pose_sample &)>
expected<std::unique_ptr<trajectory::pose_trajectory_generator>, refusal> not_finite_screw_pose_waypoints(std::span<const transform> waypoints, const transform &seed,
                                                                                                          double max_linear_speed, double max_angular_speed)
{
    return spoiling<trajectory::pose_trajectory_generator, trajectory::pose_sample, Spoil, &inside>(
            trajectory::screw_pose_waypoints(waypoints, seed, max_linear_speed, max_angular_speed));
}

}

#endif
