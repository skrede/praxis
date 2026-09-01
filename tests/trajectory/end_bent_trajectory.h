#ifndef HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_END_BENT_TRAJECTORY_H
#define HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_END_BENT_TRAJECTORY_H

#include "praxis/trajectory/baseline/trajectory.h"

#include "praxis/trajectory/types.h"
#include "praxis/trajectory/trajectory.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <span>
#include <memory>
#include <cstdint>
#include <utility>

namespace praxis::fixture {

// Which end of the motion a binding is displaced at. A comparison that samples only the other end
// reports nothing about it.
enum class displaced_end : std::uint8_t
{
    start,
    finish,
};

// Radians, a decade above the bound the row is judged at.
inline constexpr double bent_at_one_end = 1.0e-3;

// The fraction of the span each end reaches over. Two evenly spaced times fall in the two bands only
// if the run samples both ends of the motion.
inline constexpr double end_band = 0.1;

class trajectory_bent_at_one_end : public trajectory::trajectory_generator
{
public:
    trajectory_bent_at_one_end(std::unique_ptr<trajectory_generator> held, displaced_end at)
            : m_at(at)
            , m_held(std::move(held))
    {
    }

    expected<trajectory::trajectory_sample, refusal> sample(double t) const override
    {
        const expected<trajectory::trajectory_sample, refusal> read = m_held->sample(t);
        if(!read || !within_the_band(t))
            return read;

        trajectory::trajectory_sample moved = *read;
        moved.position[0] += bent_at_one_end;

        return moved;
    }

    double duration() const override
    {
        return m_held->duration();
    }

private:
    displaced_end m_at;
    std::unique_ptr<trajectory_generator> m_held;

    bool within_the_band(double t) const
    {
        const double span = m_held->duration();

        return m_at == displaced_end::start ? t <= end_band * span : t >= (1.0 - end_band) * span;
    }
};

inline expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> bent_at(displaced_end at, std::span<const trajectory::configuration> waypoints,
                                                                                    const trajectory::configuration &j0, const trajectory::configuration_limits &limits)
{
    expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> answered = trajectory::joint_space_waypoints(waypoints, j0, limits);
    if(!answered)
        return unexpected(answered.error());

    return std::unique_ptr<trajectory::trajectory_generator>(std::make_unique<trajectory_bent_at_one_end>(std::move(*answered), at));
}

inline expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> bent_at_the_start(std::span<const trajectory::configuration> waypoints, const trajectory::configuration &j0,
                                                                                              const trajectory::configuration_limits &limits)
{
    return bent_at(displaced_end::start, waypoints, j0, limits);
}

inline expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> bent_at_the_finish(std::span<const trajectory::configuration> waypoints, const trajectory::configuration &j0,
                                                                                               const trajectory::configuration_limits &limits)
{
    return bent_at(displaced_end::finish, waypoints, j0, limits);
}

}

#endif
