#include "trajectory_preview.h"

#include <span>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::manipulator {

namespace {

// Samples per previewed motion, over the whole of its duration.
constexpr std::size_t preview_samples = 385;

double instant(double span, std::size_t at)
{
    return span * static_cast<double>(at) / static_cast<double>(preview_samples - 1);
}

expected<std::vector<preview_sample>, refusal> along(const trajectory::trajectory_generator &motion, const scene_robot &arm, double span)
{
    std::vector<preview_sample> taken;
    taken.reserve(preview_samples);

    for(std::size_t at = 0; at < preview_samples; ++at)
    {
        const double t                                             = instant(span, at);
        const expected<trajectory::trajectory_sample, refusal> one = motion.sample(t);
        if(!one)
            return unexpected(one.error());

        taken.push_back(preview_sample{t, *one, arm.tool_pose_at(one->position)});
    }

    return taken;
}

// A scaling the composition left unbound refuses, and its curve is left empty rather than costing
// the preview the curves standing beside it.
std::vector<trajectory::scaling_sample> curve_of(const prepared_time_scaling &scaled, double span)
{
    std::vector<trajectory::scaling_sample> curve;
    curve.reserve(preview_samples);

    for(std::size_t at = 0; at < preview_samples; ++at)
    {
        const expected<trajectory::scaling_sample, refusal> one = scaled.sample(instant(span, at), span);
        if(!one)
            return {};

        curve.push_back(*one);
    }

    return curve;
}

// The distance from the first sample to each of them in turn, the last entry being the whole. One
// accumulation rather than two, so the last entry over the whole is exactly one.
std::vector<double> walked(std::span<const preview_sample> taken)
{
    std::vector<double> reached;
    reached.reserve(taken.size());
    reached.push_back(0.0);

    for(std::size_t at = 1u; at < taken.size(); ++at)
        reached.push_back(reached.back() + (taken[at].motion.position - taken[at - 1u].motion.position).norm());

    return reached;
}

// Where the joint rate vanishes the quotient below has no answer, and the derivative of the rate
// approaches the acceleration's own magnitude from either side of such an instant, signed by the
// side the run carries on down.
double approached(std::span<const preview_sample> taken, std::size_t at, double total)
{
    const std::size_t last   = taken.size() - 1u;
    const std::size_t before = at == 0u ? 0u : at - 1u;
    const std::size_t after  = at == last ? last : at + 1u;
    const double magnitude   = taken[at].motion.acceleration.norm() / total;

    return taken[after].motion.velocity.norm() >= taken[before].motion.velocity.norm() ? magnitude : -magnitude;
}

// dq/dt = q'(s) s'(t) and d2q/dt2 = q''(s) s'(t)^2 + q'(s) s''(t), so the rate of the parameter is
// the joint rate's magnitude over the whole distance and its own derivative is the quotient those
// two identities leave: Lynch & Park, Modern Robotics, eq. (9.1) and (9.2).
trajectory::scaling_sample realized_at(std::span<const preview_sample> taken, double reached, std::size_t at, double total)
{
    const double rate  = taken[at].motion.velocity.norm();
    const double slope = rate > 0.0 ? taken[at].motion.velocity.dot(taken[at].motion.acceleration) / (rate * total) : approached(taken, at, total);

    return trajectory::scaling_sample{reached / total, rate / total, slope};
}

}

expected<preview_run, refusal> sampled_preview(const trajectory::trajectory_generator &motion, const scene_robot &arm, std::span<const prepared_time_scaling> scalings)
{
    const double span                                    = motion.duration();
    expected<std::vector<preview_sample>, refusal> taken = along(motion, arm, span);
    if(!taken)
        return unexpected(taken.error());

    std::vector<std::vector<trajectory::scaling_sample>> curves;
    curves.reserve(scalings.size());
    for(const prepared_time_scaling &scaled : scalings)
        curves.push_back(curve_of(scaled, span));

    std::vector<trajectory::scaling_sample> realized = realized_parameter(*taken);

    return preview_run{span, std::move(*taken), std::move(curves), std::move(realized)};
}

std::vector<trajectory::scaling_sample> realized_parameter(std::span<const preview_sample> taken)
{
    if(taken.size() < 2u)
        return {};

    const std::vector<double> reached = walked(taken);
    const double total                = reached.back();
    if(!(total > 0.0))
        return {};

    std::vector<trajectory::scaling_sample> curve;
    curve.reserve(taken.size());
    for(std::size_t at = 0; at < taken.size(); ++at)
        curve.push_back(realized_at(taken, reached[at], at, total));

    return curve;
}

}
