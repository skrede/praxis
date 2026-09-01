#include "praxis/manipulator/motion_commands.h"

#include "praxis/trajectory/detail/finite_difference.h"

#include "praxis/evaluation/tolerance.h"

#include <spdlog/spdlog.h>

#include <memory>
#include <utility>
#include <algorithm>
#include <functional>

namespace praxis::manipulator {

namespace {

using joint_path = std::function<expected<joint_vector, refusal>(double s)>;

// A resolved path starts at the arm's configuration only to the solver's tolerance, so the check
// that a motion begins where the arm is has to be loose.
constexpr double start_agreement = 1.0e-3;

class composed_trajectory : public trajectory::trajectory_generator
{
public:
    composed_trajectory(joint_path shape, const prepared_time_scaling &shaped_in_time, double span)
            : m_span(span)
            , m_shape(std::move(shape))
            , m_scaling(shaped_in_time)
    {
    }

    expected<trajectory::trajectory_sample, refusal> sample(double t) const override
    {
        const expected<trajectory::scaling_sample, refusal> scaled = m_scaling.sample(t, m_span);
        if(!scaled)
            return unexpected(scaled.error());

        const double s                                                               = std::clamp(scaled->s, 0.0, 1.0);
        const expected<trajectory::detail::derivatives<joint_vector>, refusal> along = trajectory::detail::central_differences(m_shape, s);
        const expected<joint_vector, refusal> reached                                = m_shape(s);
        if(!along)
            return unexpected(along.error());
        if(!reached)
            return unexpected(reached.error());

        // dq/dt = q'(s) s'(t) and d2q/dt2 = q''(s) s'(t)^2 + q'(s) s''(t): Lynch & Park, Modern
        // Robotics, eq. (9.1) and (9.2). The path's derivatives are the numerical ones above.
        return trajectory::trajectory_sample{*reached, along->first_derivative * scaled->ds, along->second_derivative * scaled->ds * scaled->ds + along->first_derivative * scaled->dds};
    }

    double duration() const override
    {
        return m_span;
    }

private:
    double m_span;
    joint_path m_shape;
    prepared_time_scaling m_scaling;
};

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> compose(const prepared_time_scaling &scaled, const joint_limits &bounds, const joint_vector &start,
                                                                             joint_path shape)
{
    const expected<joint_vector, refusal> begins = shape(0.0);
    const expected<joint_vector, refusal> ends   = shape(1.0);
    if(!begins || !ends)
    {
        spdlog::error("praxis: the composed path refused at an endpoint, so it carries no motion");

        return unexpected(begins ? ends.error() : begins.error());
    }
    if(!is_approx_equal(*begins, start, start_agreement))
    {
        spdlog::error("praxis: the composed path does not begin where the arm is, so it carries no motion");

        return unexpected(refusal::unsupported_input);
    }

    // The trapezoid's bounds follow from the motion's extent, which is known here and nowhere
    // earlier, and the duration is taken from the same pair the profile is later sampled against.
    const prepared_time_scaling held(scaled, scaled.held_to().value_or(derived_bounds(bounds, *begins, *ends)));

    return std::make_unique<composed_trajectory>(std::move(shape), held, held.duration(bounds, *begins, *ends));
}

}

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> joint_space_motion(const trajectory::path_ops &injected_path, const prepared_time_scaling &scaled,
                                                                                        const joint_limits &bounds, const joint_vector &start, const joint_vector &target)
{
    const auto straight = injected_path.joint_straight_line;

    return compose(scaled, bounds, start, [straight, start, target](double s) { return straight(start, target, s); });
}

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> task_space_motion(const motion_ops &injected_motion, const prepared_time_scaling &scaled, const kinematics &solver,
                                                                                       const joint_limits &bounds, const joint_vector &start, const transform &start_pose,
                                                                                       const transform &end_pose, task_space_path shape)
{
    const auto resolve = injected_motion.task_space_pose;
    const std::reference_wrapper<const kinematics> borrowed(solver);

    return compose(scaled, bounds, start,
                   [resolve, borrowed, shape, start_pose, end_pose, start](double s) -> expected<joint_vector, refusal>
                   {
                       const expected<transform, refusal> along = shape(start_pose, end_pose, s);
                       if(!along)
                           return unexpected(along.error());

                       return resolve(borrowed, *along, start);
                   });
}

}
