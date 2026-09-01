#include "fixtures.h"

#include "answered.h"

#include "praxis/manipulator/motion_commands.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <algorithm>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

// A path with a nonzero second derivative in the path parameter, which is what makes both terms of
// the acceleration identity observable: a straight line passes with the curvature term missing.
expected<joint_vector, refusal> quadratic_blend(const joint_vector &start, const joint_vector &end, double s)
{
    return joint_vector(start + s * s * (end - start));
}

trajectory::path_ops curved_path()
{
    trajectory::path_ops shapes = composing_path();
    shapes.joint_straight_line  = &quadratic_blend;

    return shapes;
}

// A path that answers everywhere, and everywhere one unit away from the configuration it was handed.
expected<joint_vector, refusal> displaced_line(const joint_vector &start, const joint_vector &end, double s)
{
    return joint_vector(start + joint_vector::Constant(start.size(), 1.0) + s * (end - start));
}

trajectory::path_ops displaced_path()
{
    trajectory::path_ops shapes = composing_path();
    shapes.joint_straight_line  = &displaced_line;

    return shapes;
}

joint_limits bounds()
{
    joint_limits limits{};
    limits.velocity     = joint_vector::Constant(2, 0.5);
    limits.acceleration = joint_vector::Constant(2, 2.0);

    return limits;
}

trajectory::trajectory_sample at(const trajectory::trajectory_generator &motion, double t)
{
    auto sampled = motion.sample(t);
    REQUIRE(sampled);

    return *sampled;
}

const joint_vector from = configuration(0.1, -0.2);
const joint_vector to   = configuration(0.9, 0.6);

// The duration the quintic rule gave this pair before a scaling could be chosen: 15/8 of the 0.8
// each joint moves over the 0.5 per second it may move, which stands above both the acceleration
// bound and the floor.
constexpr double quintic_span = 3.0;

prepared_time_scaling as_quintic()
{
    return prepared_time_scaling(composing_time_scaling(), time_scaling_choice::quintic);
}

transform planar_pose(const joint_vector &q)
{
    transform pose = transform::Identity();
    pose(0, 3)     = q[0];
    pose(1, 3)     = q[1];

    return pose;
}

// The farther joint decides both derived bounds, so the reach below is what places a pair on one
// side of the trapezoid's branch switch or the other: the coast survives from 0.125 upwards.
joint_vector reaching(double reach)
{
    return joint_vector(from + configuration(reach, 0.5 * reach));
}

constexpr std::array<double, 12> reaches{0.02, 0.05, 0.08, 0.1, 0.125, 0.15, 0.2, 0.4, 0.8, 1.2, 1.6, 2.0};

double widest_gap(const prepared_time_scaling &scaled, double span, double first, double last)
{
    double lowest  = std::numeric_limits<double>::max();
    double highest = std::numeric_limits<double>::lowest();
    for(int i = 0; i <= 20; ++i)
    {
        const double t   = first + (last - first) * static_cast<double>(i) / 20.0;
        const double dds = answered(scaled.sample(t, span)).dds;
        lowest           = std::min(lowest, dds);
        highest          = std::max(highest, dds);
    }

    return highest - lowest;
}

double largest_magnitude(const prepared_time_scaling &scaled, double span, double first, double last)
{
    double largest = 0.0;
    for(int i = 0; i <= 20; ++i)
    {
        const double t = first + (last - first) * static_cast<double>(i) / 20.0;
        largest        = std::max(largest, std::abs(answered(scaled.sample(t, span)).dds));
    }

    return largest;
}

}

TEST_CASE("a_joint_space_motion_runs_from_the_arms_configuration_to_the_target")
{
    const auto motion = joint_space_motion(composing_path(), as_quintic(), bounds(), from, to);
    REQUIRE(motion.has_value());

    const double span = (*motion)->duration();

    REQUIRE(span > 0.0);
    CHECK(is_approx_equal(at(**motion, 0.0).position, from));
    CHECK(is_approx_equal(at(**motion, span).position, to, 1.0e-9));
    CHECK(at(**motion, 0.0).velocity.isZero(default_tolerance));
    CHECK(at(**motion, span).velocity.isZero(1.0e-9));
    CHECK(!at(**motion, 0.5 * span).velocity.isZero(1.0e-6));
}

TEST_CASE("sampling_is_a_pure_function_of_the_time_it_is_asked_for")
{
    const auto motion = joint_space_motion(composing_path(), as_quintic(), bounds(), from, to);
    REQUIRE(motion.has_value());

    const double span       = (*motion)->duration();
    const joint_vector late = at(**motion, 0.7 * span).position;

    CHECK(is_approx_equal(at(**motion, 0.2 * span).position, at(**motion, 0.2 * span).position));
    CHECK(is_approx_equal(at(**motion, 0.7 * span).position, late));
}

TEST_CASE("the_sample_carries_the_chain_rule_and_not_only_its_first_term")
{
    const auto motion = joint_space_motion(curved_path(), as_quintic(), bounds(), from, to);
    REQUIRE(motion.has_value());

    const double span                       = (*motion)->duration();
    const double t                          = 0.3 * span;
    const trajectory::scaling_sample scaled = answered(quintic(t, span));

    // q(s) = from + s^2 (to - from), so q'(s) = 2 s (to - from) and q''(s) = 2 (to - from).
    const joint_vector reach  = to - from;
    const joint_vector first  = 2.0 * scaled.s * reach;
    const joint_vector second = 2.0 * reach;

    const trajectory::trajectory_sample sampled = at(**motion, t);
    CHECK((sampled.velocity - first * scaled.ds).isZero(1.0e-6));
    CHECK((sampled.acceleration - (second * scaled.ds * scaled.ds + first * scaled.dds)).isZero(1.0e-5));
    CHECK((first * scaled.dds).cwiseAbs().maxCoeff() > 0.1 * sampled.acceleration.cwiseAbs().maxCoeff());
}

TEST_CASE("the_duration_keeps_every_joint_inside_its_velocity_and_acceleration_bounds")
{
    const auto motion = joint_space_motion(composing_path(), as_quintic(), bounds(), from, to);
    REQUIRE(motion.has_value());

    const double span = (*motion)->duration();

    double fastest = 0.0;
    double hardest = 0.0;
    for(int i = 0; i <= 100; ++i)
    {
        const trajectory::trajectory_sample sampled = at(**motion, span * static_cast<double>(i) / 100.0);
        fastest                                     = std::max(fastest, sampled.velocity.cwiseAbs().maxCoeff());
        hardest                                     = std::max(hardest, sampled.acceleration.cwiseAbs().maxCoeff());
    }

    CHECK(fastest <= bounds().velocity[0] + 1.0e-6);
    CHECK(hardest <= bounds().acceleration[0] + 1.0e-6);
    CHECK(fastest > 0.5 * bounds().velocity[0]);
}

// What every scenario opened at before the choice existed, so a difference here is a scenario whose
// motion changed under it.
TEST_CASE("the_quintic_choice_answers_the_duration_and_the_samples_the_composition_answered_before")
{
    const auto motion = joint_space_motion(composing_path(), as_quintic(), bounds(), from, to);
    REQUIRE(motion.has_value());

    CHECK(std::abs((*motion)->duration() - quintic_span) <= 1.0e-12);
    for(const double part : {0.2, 0.4, 0.6, 0.8})
    {
        const double t                          = part * quintic_span;
        const trajectory::scaling_sample scaled = answered(quintic(t, quintic_span));
        const joint_vector reached              = joint_vector(from + scaled.s * (to - from));
        INFO("at " << t);
        CHECK(is_approx_equal(at(**motion, t).position, reached, 1.0e-12));
    }
}

// The bound profile is rescaled to whatever duration it is handed and rescaling only stretches, so a
// duration below the profile's own is refused rather than slowed down.
TEST_CASE("the_trapezoidal_choice_hands_the_profile_a_duration_it_accepts_over_both_branches")
{
    const prepared_time_scaling scaled(trajectory::baseline().time_scaling, time_scaling_choice::trapezoidal);
    for(const double reach : reaches)
    {
        const joint_vector target        = reaching(reach);
        const path_parameter_bounds held = derived_bounds(bounds(), from, target);
        const double span                = scaled.duration(bounds(), from, target);
        INFO("reach " << reach << " rate " << held.max_rate << " rate change " << held.max_rate_change << " span " << span);

        const auto motion = joint_space_motion(composing_path(), scaled, bounds(), from, target);
        REQUIRE(motion.has_value());
        for(int i = 0; i <= 10; ++i)
            REQUIRE((*motion)->sample(span * static_cast<double>(i) / 10.0).has_value());
    }
}

// A duration merely long enough would pass the case above, so each branch is also asked for the
// largest duration the profile refuses.
TEST_CASE("the_trapezoidal_duration_is_the_profiles_own_and_not_merely_long_enough")
{
    const trajectory::time_scaling_ops reference = trajectory::baseline().time_scaling;
    const prepared_time_scaling scaled(reference, time_scaling_choice::trapezoidal);
    for(const double reach : {0.08, 0.8})
    {
        const joint_vector target        = reaching(reach);
        const path_parameter_bounds held = derived_bounds(bounds(), from, target);
        const double span                = scaled.duration(bounds(), from, target);
        INFO("reach " << reach << " span " << span);

        CHECK(reference.trapezoidal(0.0, span, held.max_rate, held.max_rate_change).has_value());
        CHECK_FALSE(reference.trapezoidal(0.0, 0.999 * span, held.max_rate, held.max_rate_change).has_value());
    }
}

// Where the rate is exactly the peak the rate of change reaches, the coast has collapsed and the
// residual displacement it would be taken from rounds below zero, which is the one place a duration
// written in any other arrangement of the same algebra lands short of the profile's own.
TEST_CASE("a_trapezoid_whose_coast_has_exactly_collapsed_is_still_a_duration_the_profile_accepts")
{
    const trajectory::time_scaling_ops reference = trajectory::baseline().time_scaling;
    for(const double rate_change : {0.05, 0.5, 5.0, 50.0})
    {
        const path_parameter_bounds held{std::sqrt(rate_change), rate_change};
        const prepared_time_scaling scaled(reference, time_scaling_choice::trapezoidal, held);
        const double span = scaled.duration(bounds(), from, to);
        INFO("rate change " << rate_change << " span " << span);

        CHECK(scaled.sample(0.5 * span, span).has_value());
    }
}

// The picture Lynch & Park's section on the trapezoidal profile is about: an acceleration that steps
// between three constants against one that never stops turning.
TEST_CASE("a_trapezoidal_motion_accelerates_in_steps_where_a_quintic_one_curves")
{
    const trajectory::time_scaling_ops reference = trajectory::baseline().time_scaling;
    const path_parameter_bounds held             = derived_bounds(bounds(), from, to);
    const prepared_time_scaling trapezoid(reference, time_scaling_choice::trapezoidal, held);
    const prepared_time_scaling smooth(reference, time_scaling_choice::quintic);

    // Each ramp lasts the rate over the rate of change, so the coast is what lies between them.
    const double span = trapezoid.duration(bounds(), from, to);
    const double ramp = held.max_rate / held.max_rate_change;

    CHECK(widest_gap(trapezoid, span, 0.1 * ramp, 0.9 * ramp) <= 1.0e-9);
    CHECK(largest_magnitude(trapezoid, span, 1.1 * ramp, span - 1.1 * ramp) <= 1.0e-9);
    CHECK(widest_gap(smooth, span, 0.1 * ramp, 0.9 * ramp) > 1.0e-3);
    CHECK(largest_magnitude(smooth, span, 1.1 * ramp, span - 1.1 * ramp) > 1.0e-3);
}

// An override is the one thing that moves the duration off what the arm's own limits give, and it
// moves it in the direction the looser or tighter pair asks for.
TEST_CASE("an_overridden_pair_of_bounds_lengthens_or_shortens_the_motion_against_the_derived_one")
{
    const trajectory::time_scaling_ops reference = trajectory::baseline().time_scaling;
    const path_parameter_bounds held             = derived_bounds(bounds(), from, to);
    const prepared_time_scaling derived(reference, time_scaling_choice::trapezoidal);
    const prepared_time_scaling looser(reference, time_scaling_choice::trapezoidal, path_parameter_bounds{2.0 * held.max_rate, 2.0 * held.max_rate_change});
    const prepared_time_scaling tighter(reference, time_scaling_choice::trapezoidal, path_parameter_bounds{0.5 * held.max_rate, 0.5 * held.max_rate_change});

    CHECK(looser.duration(bounds(), from, to) < derived.duration(bounds(), from, to));
    CHECK(tighter.duration(bounds(), from, to) > derived.duration(bounds(), from, to));
    CHECK(derived.held_to() == std::nullopt);
    CHECK(looser.chosen() == time_scaling_choice::trapezoidal);
}

TEST_CASE("a_task_space_motion_follows_the_path_the_caller_supplies")
{
    const kinematics solver = sliding_solver();
    const auto motion       = task_space_motion(composing_motion(), as_quintic(), solver, bounds(), from, planar_pose(from), planar_pose(to), &interpolated);
    REQUIRE(motion.has_value());

    REQUIRE((*motion)->duration() > 0.0);
    CHECK(is_approx_equal(at(**motion, 0.0).position, from, 1.0e-9));
    CHECK(is_approx_equal(at(**motion, (*motion)->duration()).position, to, 1.0e-9));
}

// Every slot on its inert default, so the path refuses at both endpoints and the refusal is the one
// the path produced rather than a shape invented here.
TEST_CASE("a_composition_whose_path_refuses_at_an_endpoint_carries_that_refusal_and_no_motion")
{
    const prepared_time_scaling inert(trajectory::time_scaling_ops{}, time_scaling_choice::quintic);
    const auto motion = joint_space_motion(trajectory::path_ops{}, inert, bounds(), from, to);

    REQUIRE_FALSE(motion.has_value());
    CHECK(motion.error() == refusal::not_implemented);
}

// A path that answers, but somewhere other than where the arm is, would command a jump the operator
// never asked for; the request shape is not one this composition serves.
TEST_CASE("a_composition_that_does_not_begin_where_the_arm_is_is_refused_rather_than_commanded")
{
    const auto motion    = joint_space_motion(composing_path(), as_quintic(), bounds(), from, to);
    const auto elsewhere = joint_space_motion(displaced_path(), as_quintic(), bounds(), from, to);

    REQUIRE(motion.has_value());
    REQUIRE_FALSE(elsewhere.has_value());
    CHECK(elsewhere.error() == refusal::unsupported_input);
}
