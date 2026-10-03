#ifndef HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_CONFIGURATION_RUN_H
#define HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_CONFIGURATION_RUN_H

#include "praxis/trajectory/baseline/trajectory.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::fixture {

using namespace trajectory;

inline configuration pair_of(double first, double second)
{
    configuration q(2);
    q << first, second;

    return q;
}

inline configuration_limits bounds()
{
    configuration_limits limits{};
    limits.velocity     = pair_of(1.0, 0.5);
    limits.acceleration = pair_of(2.0, 2.0);

    return limits;
}

inline std::unique_ptr<trajectory_generator> commanded(std::span<const configuration> via, const configuration &seed)
{
    auto motion = joint_space_waypoints(via, seed, bounds());
    REQUIRE(motion);

    return std::move(*motion);
}

inline trajectory_sample at(const trajectory_generator &motion, double t)
{
    auto sampled = motion.sample(t);
    REQUIRE(sampled);

    return *sampled;
}

// The apportionment the reference is held to, written out here rather than taken from it, so a
// change to either is a failure rather than a silent agreement. Every segment carries the same share
// of whatever the run was stretched to.
inline std::vector<double> knots(std::span<const configuration> via, double stretched_to)
{
    const configuration velocity = bounds().velocity;
    std::vector<double> times    = {0.0};

    for(std::size_t k = 1u; k < via.size(); ++k)
    {
        double span = 0.0;
        for(Eigen::Index i = 0; i < via[k].size(); ++i)
            span = std::max(span, std::abs(via[k][i] - via[k - 1u][i]) / velocity[i]);

        times.push_back(times.back() + span);
    }

    for(double &knot : times)
        knot *= stretched_to / times.back();

    return times;
}

// Biagiotti & Melchiorri, Trajectory Planning for Automatic Machines and Robots, sec. 2.1.4,
// eq. (2.3) at row k of a run timed by `times`, with both ends at rest.
inline configuration book_velocity(std::span<const configuration> via, const std::vector<double> &times, std::size_t k)
{
    configuration velocity = configuration::Zero(via[k].size());
    for(Eigen::Index i = 0; k > 0u && k + 1u < via.size() && i < velocity.size(); ++i)
    {
        const double before = (via[k][i] - via[k - 1u][i]) / (times[k] - times[k - 1u]);
        const double after  = (via[k + 1u][i] - via[k][i]) / (times[k + 1u] - times[k]);
        const bool agree    = (before > 0.0 && after > 0.0) || (before < 0.0 && after < 0.0) || (before == 0.0 && after == 0.0);
        velocity[i]         = agree ? 0.5 * (before + after) : 0.0;
    }

    return velocity;
}

}

#endif
