#include "swept_pose.h"

#include "praxis/trajectory/baseline/path.h"
#include "praxis/trajectory/baseline/pose_trajectory.h"

#include <spdlog/spdlog.h>

#include <memory>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::trajectory {

namespace {

// A segment reports its twists in its own start frame, so a sample is read against the pose the
// segment holding that instant begins at. Every segment carries a scaling of its own whose rate is
// zero at both ends, so the motion stands still at each pose it is carried through.
class carried_pose_trajectory : public pose_trajectory_generator
{
public:
    explicit carried_pose_trajectory(std::vector<std::unique_ptr<pose_trajectory_generator>> segments)
            : m_span(0.0)
            , m_segments(std::move(segments))
    {
        m_starts.reserve(m_segments.size());
        for(const std::unique_ptr<pose_trajectory_generator> &segment : m_segments)
        {
            m_starts.push_back(m_span);
            m_span += segment->duration();
        }
    }

    expected<pose_sample, refusal> sample(double t) const override
    {
        const double held    = std::min(t, m_span);
        const auto after     = std::upper_bound(m_starts.begin(), m_starts.end(), held);
        const std::size_t at = after == m_starts.begin() ? 0u : static_cast<std::size_t>(after - m_starts.begin()) - 1u;

        return m_segments[at]->sample(held - m_starts[at]);
    }

    double duration() const override
    {
        return m_span;
    }

private:
    double m_span;
    std::vector<double> m_starts;
    std::vector<std::unique_ptr<pose_trajectory_generator>> m_segments;
};

expected<std::unique_ptr<pose_trajectory_generator>, refusal> chained(task_space_path along, std::span<const transform> waypoints, double max_linear_speed, double max_angular_speed)
{
    std::vector<std::unique_ptr<pose_trajectory_generator>> segments;
    segments.reserve(waypoints.size() - 1u);

    for(std::size_t k = 1u; k < waypoints.size(); ++k)
    {
        expected<std::unique_ptr<pose_trajectory_generator>, refusal> segment = swept_pose(along, waypoints[k - 1u], waypoints[k], max_linear_speed, max_angular_speed);
        if(!segment)
        {
            spdlog::error("praxis: the pair of poses ending at row {} carries no motion between them, and the run stands as it was given", k);
            return unexpected(segment.error());
        }

        segments.push_back(std::move(*segment));
    }

    return std::make_unique<carried_pose_trajectory>(std::move(segments));
}

expected<std::unique_ptr<pose_trajectory_generator>, refusal> carried(task_space_path along, std::span<const transform> waypoints, const transform &seed, double max_linear_speed,
                                                                      double max_angular_speed)
{
    if(waypoints.size() == 1u)
        return swept_pose(along, seed, waypoints.front(), max_linear_speed, max_angular_speed);
    if(waypoints.size() == 2u)
        return swept_pose(along, waypoints.front(), waypoints.back(), max_linear_speed, max_angular_speed);
    if(waypoints.size() >= 3u)
        return chained(along, waypoints, max_linear_speed, max_angular_speed);

    spdlog::error("praxis: a task-space path carries at least one pose and none were given");

    return unexpected(refusal::unsupported_input);
}

}

expected<std::unique_ptr<pose_trajectory_generator>, refusal> decoupled_pose_waypoints(std::span<const transform> waypoints, const transform &seed, double max_linear_speed,
                                                                                       double max_angular_speed)
{
    return carried(&decoupled, waypoints, seed, max_linear_speed, max_angular_speed);
}

expected<std::unique_ptr<pose_trajectory_generator>, refusal> screw_pose_waypoints(std::span<const transform> waypoints, const transform &seed, double max_linear_speed,
                                                                                   double max_angular_speed)
{
    return carried(&screw, waypoints, seed, max_linear_speed, max_angular_speed);
}

}
