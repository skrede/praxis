#include "via_point_cubics.h"

#include <cmath>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::trajectory {

namespace {

class via_point_trajectory : public trajectory_generator
{
public:
    explicit via_point_trajectory(via_point_run run)
            : m_run(std::move(run))
    {
    }

    expected<trajectory_sample, refusal> sample(double t) const override
    {
        const double held                        = std::max(m_run.knots.front(), std::min(t, m_run.knots.back()));
        const std::size_t segment                = segment_at(held);
        const double dt                          = held - m_run.knots[segment];
        const segment_coefficient_block &reached = m_run.segments[segment];

        configuration position     = reached.col(0) + dt * (reached.col(1) + dt * (reached.col(2) + dt * reached.col(3)));
        configuration velocity     = reached.col(1) + dt * (2.0 * reached.col(2) + 3.0 * dt * reached.col(3));
        configuration acceleration = 2.0 * reached.col(2) + 6.0 * dt * reached.col(3);

        return trajectory_sample{std::move(position), std::move(velocity), std::move(acceleration)};
    }

    double duration() const override
    {
        return m_run.knots.back();
    }

private:
    via_point_run m_run;

    std::size_t segment_at(double held) const
    {
        const auto after = std::upper_bound(m_run.knots.begin(), m_run.knots.end(), held);

        return std::min(static_cast<std::size_t>(after - m_run.knots.begin()), m_run.segments.size()) - 1u;
    }
};

configuration velocities_at(std::span<const configuration> waypoints, const std::vector<double> &knots, std::size_t k)
{
    configuration velocity = configuration::Zero(waypoints[k].size());
    if(k == 0u || k + 1u == waypoints.size())
        return velocity;

    const configuration before = (waypoints[k] - waypoints[k - 1u]) / (knots[k] - knots[k - 1u]);
    const configuration after  = (waypoints[k + 1u] - waypoints[k]) / (knots[k + 1u] - knots[k]);
    for(Eigen::Index i = 0; i < velocity.size(); ++i)
        velocity[i] = via_velocity(before[i], after[i]);

    return velocity;
}

double segment_speed(const Eigen::Vector4d &coefficients, double dt)
{
    return std::abs(coefficients[1] + dt * (2.0 * coefficients[2] + 3.0 * dt * coefficients[3]));
}

}

double via_velocity(double before, double after)
{
    const auto sign = [](double x) { return (x > 0.0) - (x < 0.0); };

    return sign(before) != sign(after) ? 0.0 : 0.5 * (before + after);
}

Eigen::Vector4d segment_coefficients(double from, double to, double leaving, double arriving, double span)
{
    const double a2 = (3.0 * to - 3.0 * from - 2.0 * leaving * span - arriving * span) / (span * span);
    const double a3 = (2.0 * from + (leaving + arriving) * span - 2.0 * to) / (span * span * span);

    return Eigen::Vector4d(from, leaving, a2, a3);
}

via_point_run fitted_run(std::span<const configuration> waypoints, const std::vector<double> &knots)
{
    const Eigen::Index width = waypoints.front().size();
    via_point_run run{knots, {}};
    run.segments.reserve(waypoints.size() - 1u);

    configuration leaving = velocities_at(waypoints, knots, 0u);
    for(std::size_t j = 0u; j + 1u < waypoints.size(); ++j)
    {
        const configuration arriving = velocities_at(waypoints, knots, j + 1u);
        segment_coefficient_block block(width, 4);
        for(Eigen::Index i = 0; i < width; ++i)
            block.row(i) = segment_coefficients(waypoints[j][i], waypoints[j + 1u][i], leaving[i], arriving[i], knots[j + 1u] - knots[j]).transpose();

        run.segments.push_back(std::move(block));
        leaving = arriving;
    }

    return run;
}

double peak_speed(const via_point_run &run, Eigen::Index dof)
{
    double peak = 0.0;
    for(std::size_t j = 0u; j < run.segments.size(); ++j)
    {
        const Eigen::Vector4d coefficients = run.segments[j].row(dof).transpose();
        const double span                  = run.knots[j + 1u] - run.knots[j];
        peak                               = std::max({peak, segment_speed(coefficients, 0.0), segment_speed(coefficients, span)});

        const double turning = coefficients[3] != 0.0 ? -coefficients[2] / (3.0 * coefficients[3]) : 0.0;
        if(turning > 0.0 && turning < span)
            peak = std::max(peak, segment_speed(coefficients, turning));
    }

    return peak;
}

std::unique_ptr<trajectory_generator> via_point_generator(via_point_run run)
{
    return std::make_unique<via_point_trajectory>(std::move(run));
}

}
