#include "straight_line.h"

#include "praxis/trajectory/baseline/path.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <utility>
#include <algorithm>

namespace praxis::trajectory {

namespace {

// Constant speed between two configurations. The velocity it reports jumps at both ends, and the
// acceleration it reports is the zero it carries between them rather than the two impulses that
// produce those jumps.
class straight_line_trajectory : public trajectory_generator
{
public:
    straight_line_trajectory(configuration from, configuration to, double span)
            : m_span(span)
            , m_to(std::move(to))
            , m_from(std::move(from))
    {
    }

    expected<trajectory_sample, refusal> sample(double t) const override
    {
        const bool traversing    = m_span > 0.0;
        const double s           = traversing ? std::clamp(t / m_span, 0.0, 1.0) : 1.0;
        const bool moving        = traversing && t > 0.0 && t < m_span;
        const configuration rest = configuration::Zero(m_from.size());

        const expected<configuration, refusal> reached = joint_straight_line(m_from, m_to, s);
        if(!reached)
            return unexpected(reached.error());

        return trajectory_sample{*reached, moving ? configuration((m_to - m_from) / m_span) : rest, rest};
    }

    double duration() const override
    {
        return m_span;
    }

private:
    double m_span;
    configuration m_to;
    configuration m_from;
};

}

double traversal_time(const configuration &from, const configuration &to, const configuration &velocity)
{
    double span = 0.0;
    for(Eigen::Index i = 0; i < std::min(from.size(), to.size()); ++i)
        if(i < velocity.size() && velocity[i] > 0.0)
            span = std::max(span, std::abs(to[i] - from[i]) / velocity[i]);

    return span;
}

expected<std::unique_ptr<trajectory_generator>, refusal> straight_line(const configuration &from, const configuration &to, const configuration_limits &limits)
{
    if(from.size() != to.size())
    {
        spdlog::error("praxis: a straight line joins two configurations of one size and was given {} and {}", from.size(), to.size());
        return unexpected(refusal::unsupported_input);
    }

    return std::make_unique<straight_line_trajectory>(from, to, traversal_time(from, to, limits.velocity));
}

}
