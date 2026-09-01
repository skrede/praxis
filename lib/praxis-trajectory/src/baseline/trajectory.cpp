#include "straight_line.h"

#include "praxis/trajectory/baseline/trajectory.h"

#include <ctrlpp/trajectory/cubic_spline.h>

#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::trajectory {

namespace {

// One cubic per degree of freedom over one shared vector of knot times, so every degree of freedom
// stands at every via point at the same instant. Lynch & Park, Modern Robotics, sec. 9.3.
class via_point_trajectory : public trajectory_generator
{
public:
    via_point_trajectory(std::vector<ctrlpp::cubic_spline<double>> joints, double span)
            : m_span(span)
            , m_joints(std::move(joints))
    {
    }

    expected<trajectory_sample, refusal> sample(double t) const override
    {
        const Eigen::Index width = static_cast<Eigen::Index>(m_joints.size());
        trajectory_sample reached{configuration(width), configuration(width), configuration(width)};

        for(Eigen::Index i = 0; i < width; ++i)
        {
            const auto point        = m_joints[static_cast<std::size_t>(i)].evaluate(t);
            reached.position[i]     = point.position[0];
            reached.velocity[i]     = point.velocity[0];
            reached.acceleration[i] = point.acceleration[0];
        }

        return reached;
    }

    double duration() const override
    {
        return m_span;
    }

private:
    double m_span;
    std::vector<ctrlpp::cubic_spline<double>> m_joints;
};

std::vector<double> knot_times(std::span<const configuration> waypoints, const configuration &velocity)
{
    std::vector<double> times = {0.0};
    times.reserve(waypoints.size());

    for(std::size_t k = 1u; k < waypoints.size(); ++k)
        times.push_back(times.back() + traversal_time(waypoints[k - 1u], waypoints[k], velocity));

    return times;
}

bool widths_agree(std::span<const configuration> waypoints)
{
    for(std::size_t k = 1u; k < waypoints.size(); ++k)
        if(waypoints[k].size() != waypoints.front().size())
        {
            spdlog::error("praxis: a run of waypoints joins configurations of one size, row {} holds {} where the first holds {}, and the run stands as it was given", k,
                          waypoints[k].size(), waypoints.front().size());
            return false;
        }

    return true;
}

bool rows_advance(const std::vector<double> &times)
{
    for(std::size_t k = 1u; k < times.size(); ++k)
        if(!(times[k] > times[k - 1u]))
        {
            spdlog::error("praxis: row {} repeats the configuration the run already stands at, so it traverses in no time, and the run stands as it was given", k);
            return false;
        }

    return true;
}

expected<std::vector<ctrlpp::cubic_spline<double>>, refusal> fitted_joints(std::span<const configuration> waypoints, const std::vector<double> &times)
{
    std::vector<ctrlpp::cubic_spline<double>> joints;
    joints.reserve(static_cast<std::size_t>(waypoints.front().size()));

    for(Eigen::Index i = 0; i < waypoints.front().size(); ++i)
    {
        std::vector<double> reached;
        reached.reserve(waypoints.size());
        for(const configuration &row : waypoints)
            reached.push_back(row[i]);

        auto fitted = ctrlpp::cubic_spline<double>::create({.times = times, .positions = std::move(reached), .bc = ctrlpp::boundary_condition::clamped, .v0 = 0.0, .vn = 0.0});
        if(!fitted)
        {
            spdlog::error("praxis: the run of waypoints given carries no cubic through degree of freedom {}, and the run stands as it was given", i);
            return unexpected(refusal::unsupported_input);
        }

        joints.push_back(std::move(*fitted));
    }

    return joints;
}

// A cubic passed through a via point at speed overshoots the mean rate of the segments meeting there,
// so the largest rate reached is found where the acceleration crosses zero, the acceleration being
// linear across a segment.
double peak_rate(const ctrlpp::cubic_spline<double> &joint, const std::vector<double> &times)
{
    double peak = 0.0;
    for(std::size_t k = 1u; k < times.size(); ++k)
    {
        const auto from = joint.evaluate(times[k - 1u]);
        const auto to   = joint.evaluate(times[k]);
        peak            = std::max({peak, std::abs(from.velocity[0]), std::abs(to.velocity[0])});

        const double turning = from.acceleration[0] - to.acceleration[0];
        const double crosses = turning != 0.0 ? from.acceleration[0] / turning : 0.0;
        if(crosses > 0.0 && crosses < 1.0)
            peak = std::max(peak, std::abs(joint.evaluate(times[k - 1u] + crosses * (times[k] - times[k - 1u])).velocity[0]));
    }

    return peak;
}

// Stretching every knot time by one factor divides every rate the fit reports by that same factor,
// so the run stretched by the largest rate it reaches reaches its bounds and does not cross them.
std::vector<double> bounded_times(const std::vector<ctrlpp::cubic_spline<double>> &joints, const configuration &velocity, const std::vector<double> &times)
{
    double reached = 1.0;
    for(std::size_t i = 0u; i < joints.size(); ++i)
        if(const Eigen::Index at = static_cast<Eigen::Index>(i); velocity[at] > 0.0)
            reached = std::max(reached, peak_rate(joints[i], times) / velocity[at]);

    std::vector<double> stretched;
    stretched.reserve(times.size());
    for(double knot : times)
        stretched.push_back(knot * reached);

    return stretched;
}

expected<std::unique_ptr<trajectory_generator>, refusal> via_points(std::span<const configuration> waypoints, const configuration_limits &limits)
{
    if(!widths_agree(waypoints))
        return unexpected(refusal::unsupported_input);

    const std::vector<double> apportioned = knot_times(waypoints, limits.velocity);
    if(!rows_advance(apportioned))
        return unexpected(refusal::unsupported_input);

    expected<std::vector<ctrlpp::cubic_spline<double>>, refusal> joints = fitted_joints(waypoints, apportioned);
    if(!joints)
        return unexpected(joints.error());

    const std::vector<double> times = bounded_times(*joints, limits.velocity, apportioned);
    if(times.back() > apportioned.back())
        joints = fitted_joints(waypoints, times);
    if(!joints)
        return unexpected(joints.error());

    return std::make_unique<via_point_trajectory>(std::move(*joints), times.back());
}

}

expected<std::unique_ptr<trajectory_generator>, refusal> joint_space_waypoints(std::span<const configuration> waypoints, const configuration &j0, const configuration_limits &limits)
{
    if(waypoints.size() == 1u)
        return straight_line(j0, waypoints.front(), limits);
    if(waypoints.size() == 2u)
        return straight_line(waypoints.front(), waypoints.back(), limits);
    if(waypoints.size() >= 3u)
        return via_points(waypoints, limits);

    spdlog::error("praxis: a run of waypoints joins at least one configuration and none were given");

    return unexpected(refusal::unsupported_input);
}

}
