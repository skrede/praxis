#include "swept_pose.h"

#include "praxis/trajectory/baseline/time_scaling.h"
#include "praxis/trajectory/detail/finite_difference.h"

#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <utility>
#include <algorithm>

namespace praxis::trajectory {

namespace {

expected<double, refusal> traversal_time(const transform &from, const transform &to, double max_linear_speed, double max_angular_speed)
{
    const rotation turn = rigid_motion::rotation_matrix_from_transform(from).transpose() * rigid_motion::rotation_matrix_from_transform(to);

    const expected<std::pair<Eigen::Vector3d, double>, refusal> turned = rigid_motion::matrix_logarithm_so3(turn);
    if(!turned)
        return unexpected(turned.error());

    const double distance = (to.block<3, 1>(0, 3) - from.block<3, 1>(0, 3)).norm();
    double span           = 0.0;

    if(max_linear_speed > 0.0)
        span = std::max(span, distance / max_linear_speed);
    if(max_angular_speed > 0.0)
        span = std::max(span, std::abs(turned->second) / max_angular_speed);

    return span;
}

// Both derivatives are taken in the exponential coordinates of the motion relative to the start
// frame -- xi(s) = log(start^-1 X(s)), so that X(s) = start exp(xi(s)) and the reported twists are
// six-vectors in the start frame.
class swept_pose_trajectory : public pose_trajectory_generator
{
public:
    swept_pose_trajectory(task_space_path along, transform from, transform to, double span)
            : m_span(span)
            , m_to(std::move(to))
            , m_from(std::move(from))
            , m_reversed(rigid_motion::inverse(m_from))
            , m_along(along)
    {
    }

    expected<pose_sample, refusal> sample(double t) const override
    {
        const expected<scaling_sample, refusal> scaled = quintic(t, m_span);
        if(!scaled)
            return unexpected(scaled.error());

        const double s                                            = std::clamp(scaled->s, 0.0, 1.0);
        const expected<detail::derivatives<twist>, refusal> along = detail::central_differences([this](double u) { return coordinates(u); }, s);
        const expected<transform, refusal> placed                 = m_along(m_from, m_to, s);
        if(!along)
            return unexpected(along.error());
        if(!placed)
            return unexpected(placed.error());

        // dxi/dt = xi'(s) s'(t) and d2xi/dt2 = xi''(s) s'(t)^2 + xi'(s) s''(t): Lynch & Park, Modern
        // Robotics, eq. (9.1) and (9.2). The path's derivatives are the numerical ones above.
        return pose_sample{*placed, along->first_derivative * scaled->ds, along->second_derivative * scaled->ds * scaled->ds + along->first_derivative * scaled->dds};
    }

    double duration() const override
    {
        return m_span;
    }

private:
    double m_span;
    transform m_to;
    transform m_from;
    transform m_reversed;
    task_space_path m_along;

    expected<twist, refusal> coordinates(double s) const
    {
        const expected<transform, refusal> placed = m_along(m_from, m_to, s);
        if(!placed)
            return unexpected(placed.error());

        const expected<std::pair<screw_axis, double>, refusal> logged = rigid_motion::matrix_logarithm_se3(m_reversed * *placed);
        if(!logged)
            return unexpected(logged.error());

        return twist(logged->first * logged->second);
    }
};

}

expected<std::unique_ptr<pose_trajectory_generator>, refusal> swept_pose(task_space_path along, const transform &from, const transform &to, double max_linear_speed,
                                                                         double max_angular_speed)
{
    const expected<double, refusal> span = traversal_time(from, to, max_linear_speed, max_angular_speed);
    if(!span)
    {
        spdlog::error("praxis: the turn between the two poses given has no logarithm, so no traversal time follows from it");
        return unexpected(span.error());
    }
    if(!(*span > 0.0))
    {
        spdlog::error("praxis: the two poses given traverse in no time at a linear bound of {} and an angular bound of {}", max_linear_speed, max_angular_speed);
        return unexpected(refusal::unsupported_input);
    }

    return std::make_unique<swept_pose_trajectory>(along, from, to, *span);
}

}
