#ifndef HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_LITERAL_POSE_RUN_H
#define HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_LITERAL_POSE_RUN_H

#include "praxis/trajectory/path.h"
#include "praxis/trajectory/time_scaling.h"
#include "praxis/trajectory/pose_trajectory.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/types.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <span>
#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>
#include <iterator>
#include <algorithm>

namespace praxis::fixture {

using task_path = expected<transform, refusal> (*)(const transform &start, const transform &end, double s);

struct swept_pair
{
    transform from;
    transform to;
    double start;
    double span;
};

// A pose run written from Modern Robotics alone: each consecutive pair swept along a path under the
// quintic of eq. (9.10), over the time the slower of the two speed bounds allows, both twists taken
// of the exponential coordinates relative to the pair's start.
class literal_pose_run : public trajectory::pose_trajectory_generator
{
public:
    literal_pose_run(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, task_path along, const trajectory::time_scaling_ops &scalings,
                     std::vector<swept_pair> pairs)
            : m_screw(screw)
            , m_frames(frames)
            , m_along(along)
            , m_scalings(scalings)
            , m_pairs(std::move(pairs))
    {
    }

    // Modern Robotics eq. (9.1) and (9.2), the path's derivatives in s by central differences.
    expected<trajectory::pose_sample, refusal> sample(double t) const override
    {
        const double at        = std::clamp(t, 0.0, duration());
        const swept_pair &pair = *std::prev(std::upper_bound(std::next(m_pairs.begin()), m_pairs.end(), at, [](double u, const swept_pair &p) { return u < p.start; }));

        const expected<trajectory::scaling_sample, refusal> scaled = m_scalings.quintic(at - pair.start, pair.span);
        if(!scaled)
            return unexpected(scaled.error());

        const expected<transform, refusal> placed = m_along(pair.from, pair.to, scaled->s);
        const expected<twist, refusal> ahead      = coordinates(pair, scaled->s + step);
        const expected<twist, refusal> here       = coordinates(pair, scaled->s);
        const expected<twist, refusal> behind     = coordinates(pair, scaled->s - step);
        if(!placed || !ahead || !here || !behind)
            return unexpected(refusal::unsupported_input);

        const twist first  = (*ahead - *behind) / (2.0 * step);
        const twist second = (*ahead - 2.0 * *here + *behind) / (step * step);

        return trajectory::pose_sample{*placed, first * scaled->ds, second * scaled->ds * scaled->ds + first * scaled->dds};
    }

    double duration() const override
    {
        return m_pairs.back().start + m_pairs.back().span;
    }

private:
    static constexpr double step = 1.0e-5;

    rigid_motion::screw_ops m_screw;
    rigid_motion::frame_ops m_frames;
    task_path m_along;
    trajectory::time_scaling_ops m_scalings;
    std::vector<swept_pair> m_pairs;

    expected<twist, refusal> coordinates(const swept_pair &pair, double s) const
    {
        const expected<transform, refusal> placed = m_along(pair.from, pair.to, s);
        if(!placed)
            return unexpected(placed.error());

        const expected<std::pair<screw_axis, double>, refusal> logged = m_screw.matrix_logarithm_se3(m_frames.inverse(pair.from) * *placed);
        if(!logged)
            return unexpected(logged.error());

        return twist(logged->first * logged->second);
    }
};

inline expected<std::vector<swept_pair>, refusal> swept_pairs(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, std::span<const transform> poses,
                                                              double max_linear_speed, double max_angular_speed)
{
    std::vector<swept_pair> pairs;
    double start = 0.0;
    for(std::size_t k = 1u; k < poses.size(); ++k)
    {
        const auto turned = screw.matrix_logarithm_so3(frames.rotation_matrix_from_transform(poses[k - 1u]).transpose() * frames.rotation_matrix_from_transform(poses[k]));
        if(!turned)
            return unexpected(turned.error());

        const double span = std::max((poses[k].block<3, 1>(0, 3) - poses[k - 1u].block<3, 1>(0, 3)).norm() / max_linear_speed, std::abs(turned->second) / max_angular_speed);
        if(!(span > 0.0))
            return unexpected(refusal::unsupported_input);

        pairs.push_back(swept_pair{poses[k - 1u], poses[k], start, span});
        start += span;
    }

    return pairs;
}

inline expected<std::unique_ptr<trajectory::pose_trajectory_generator>, refusal> literal_run(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames,
                                                                                             task_path along, const trajectory::time_scaling_ops &scalings,
                                                                                             std::span<const transform> waypoints, const transform &seed, double max_linear_speed,
                                                                                             double max_angular_speed)
{
    if(waypoints.empty())
        return unexpected(refusal::unsupported_input);

    const std::vector<transform> poses = waypoints.size() == 1u ? std::vector<transform>{seed, waypoints.front()} : std::vector<transform>(waypoints.begin(), waypoints.end());
    expected<std::vector<swept_pair>, refusal> pairs = swept_pairs(screw, frames, poses, max_linear_speed, max_angular_speed);
    if(!pairs)
        return unexpected(pairs.error());

    return std::make_unique<literal_pose_run>(screw, frames, along, scalings, std::move(*pairs));
}

// The literal run along the path the slot is handed.
template<task_path trajectory::path_ops::*Along>
expected<std::unique_ptr<trajectory::pose_trajectory_generator>, refusal>
literal_pose_waypoints(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const trajectory::path_ops &path, const trajectory::time_scaling_ops &time_scaling,
                       std::span<const transform> waypoints, const transform &seed, double max_linear_speed, double max_angular_speed)
{
    return literal_run(screw, frames, path.*Along, time_scaling, waypoints, seed, max_linear_speed, max_angular_speed);
}

// The reference path, carried 0.1 m along x at its middle and nowhere at its ends.
template<task_path Along>
expected<transform, refusal> bowed(const transform &start, const transform &end, double s)
{
    expected<transform, refusal> placed = Along(start, end, s);
    if(placed)
        (*placed)(0, 3) += 0.4 * s * (1.0 - s);

    return placed;
}

// The literal run along a path of its own, whatever path the slot is handed.
template<task_path Along>
expected<std::unique_ptr<trajectory::pose_trajectory_generator>, refusal>
own_path_pose_waypoints(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const trajectory::path_ops &, const trajectory::time_scaling_ops &time_scaling,
                        std::span<const transform> waypoints, const transform &seed, double max_linear_speed, double max_angular_speed)
{
    return literal_run(screw, frames, &bowed<Along>, time_scaling, waypoints, seed, max_linear_speed, max_angular_speed);
}

}

#endif
