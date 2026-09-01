#include "answered.h"

#include "praxis/trajectory/path.h"
#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/time_scaling.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <vector>
#include <utility>
#include <algorithm>
#include <type_traits>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::trajectory;

static_assert(std::is_aggregate_v<trajectory_ops>);
static_assert(std::is_trivially_copyable_v<trajectory_ops>);
static_assert(std::is_aggregate_v<trajectory_sample>);

namespace {

// s(0) = 0 and s(T) = 1 hold, the endpoint velocities deliberately do not, and the second
// derivative is a nonzero constant so that both terms of the acceleration identity below are
// exercised.
scaling_sample ramp(double t, double duration)
{
    return scaling_sample{t * t / (duration * duration), 2.0 * t / (duration * duration), 2.0 / (duration * duration)};
}

// theta(s) = start + s^2 (end - start), sampled through a time scaling. The velocity and
// acceleration are the chain rule applied to that composition: Lynch & Park, Modern Robotics,
// eq. (9.1) and (9.2), with d(theta)/ds = 2s(end - start) and d2(theta)/ds2 = 2(end - start).
class composed_trajectory : public trajectory_generator
{
public:
    composed_trajectory(configuration start, configuration end, double motion_duration, scaling_sample (*scaling)(double, double))
            : m_duration(motion_duration)
            , m_end(std::move(end))
            , m_start(std::move(start))
            , m_scaling(scaling)
    {
    }

    expected<trajectory_sample, refusal> sample(double t) const override
    {
        scaling_sample scaling = m_scaling(std::clamp(t, 0.0, m_duration), m_duration);
        configuration span     = m_end - m_start;

        return trajectory_sample{m_start + scaling.s * scaling.s * span, 2.0 * scaling.s * scaling.ds * span,
                                 2.0 * scaling.s * scaling.dds * span + 2.0 * scaling.ds * scaling.ds * span};
    }

    double duration() const override
    {
        return m_duration;
    }

private:
    double m_duration;
    configuration m_end;
    configuration m_start;
    scaling_sample (*m_scaling)(double, double);
};

}

TEST_CASE("the_via_point_factory_slot_defaults_to_a_generator_that_refuses_to_sample")
{
    trajectory_ops ops{};
    configuration j0 = configuration::Constant(3, 0.25);
    std::vector<configuration> configurations(3, j0);

    REQUIRE(ops.joint_space_waypoints != nullptr);

    auto held = ops.joint_space_waypoints(configurations, j0, configuration_limits{});
    REQUIRE(held);

    const std::unique_ptr<trajectory_generator> generator = std::move(*held);

    REQUIRE(generator != nullptr);
    REQUIRE(is_approx_equal(generator->duration(), 0.0));

    for(double t : {-1.0, 0.0, 0.5, 1.0e6})
    {
        const expected<trajectory_sample, refusal> sampled = generator->sample(t);

        REQUIRE(!sampled);
        REQUIRE(sampled.error() == refusal::not_implemented);
    }
}

TEST_CASE("sampling_by_absolute_time_is_repeatable_and_independent_of_the_order_of_the_calls")
{
    composed_trajectory motion(configuration::Zero(3), configuration::Constant(3, 1.0), 2.0, &ramp);
    const trajectory_generator &sampled = motion;

    configuration late  = answered(sampled.sample(1.5)).position;
    configuration early = answered(sampled.sample(0.5)).position;

    REQUIRE(is_approx_equal(answered(sampled.sample(1.5)).position, late));
    REQUIRE(is_approx_equal(answered(sampled.sample(0.5)).position, early));
    REQUIRE(is_approx_equal(answered(sampled.sample(3.0)).position, answered(sampled.sample(2.0)).position));
}

TEST_CASE("a_path_composed_with_a_time_scaling_reports_the_velocity_and_acceleration_of_the_chain_rule")
{
    composed_trajectory motion(configuration::Zero(3), configuration::Constant(3, 1.0), 2.0, &ramp);
    trajectory_sample start = answered(motion.sample(0.0));
    trajectory_sample third = answered(motion.sample(1.0));
    trajectory_sample end   = answered(motion.sample(2.0));

    REQUIRE(is_approx_equal(motion.duration(), 2.0));

    REQUIRE(start.position.isZero(default_tolerance));
    REQUIRE(start.velocity.isZero(default_tolerance));

    REQUIRE(is_approx_equal(third.position, configuration::Constant(3, 0.0625)));
    REQUIRE(is_approx_equal(third.velocity, configuration::Constant(3, 0.25)));
    REQUIRE(is_approx_equal(third.acceleration, configuration::Constant(3, 0.75)));

    REQUIRE(is_approx_equal(end.position, configuration::Constant(3, 1.0)));
    REQUIRE(is_approx_equal(end.velocity, configuration::Constant(3, 2.0)));
    REQUIRE(is_approx_equal(end.acceleration, configuration::Constant(3, 3.0)));
}
