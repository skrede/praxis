#ifndef HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_CONFIGURATION_RUN_H
#define HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_CONFIGURATION_RUN_H

#include "praxis/trajectory/baseline/trajectory.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <memory>
#include <utility>

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

}

#endif
