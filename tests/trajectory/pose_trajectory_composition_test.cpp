#include "answered.h"
#include "pose_run.h"

#include "praxis/trajectory/baseline/path.h"
#include "praxis/trajectory/baseline/time_scaling.h"
#include "praxis/trajectory/baseline/pose_trajectory.h"

#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <array>
#include <memory>
#include <utility>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::trajectory;

namespace {

constexpr double time_step              = 1.0e-4;
constexpr double velocity_agreement     = 1.0e-7;
constexpr double acceleration_agreement = 1.0e-5;

// The exponential coordinates of the motion relative to the frame it is taken from: the curve the
// reported twist and its derivative are derivatives of.
twist coordinates(const transform &origin, const transform &pose)
{
    const expected<std::pair<screw_axis, double>, refusal> logged = rigid_motion::matrix_logarithm_se3(rigid_motion::inverse(origin) * pose);

    REQUIRE(logged.has_value());
    return logged->first * logged->second;
}

twist coordinates_at(const pose_trajectory_generator &motion, double t)
{
    return coordinates(seed_frame(), at(motion, t).position);
}

std::unique_ptr<pose_trajectory_generator> commanded()
{
    const std::array<transform, 1> via{waypoint()};

    auto motion = decoupled_pose_waypoints(via, seed_frame(), 0.5, 0.5);

    REQUIRE(motion);
    return std::move(*motion);
}

// Modern Robotics eq. (9.6): the rotation carried through the exponential of the scaled logarithm of
// the relative turn, the position along the chord.
transform decoupled_at(double s)
{
    const rotation from         = rigid_motion::rotation_matrix_from_transform(seed_frame());
    const auto turned           = rigid_motion::matrix_logarithm_so3(from.transpose() * rigid_motion::rotation_matrix_from_transform(waypoint()));
    const Eigen::Vector3d start = seed_frame().block<3, 1>(0, 3);
    REQUIRE(turned.has_value());
    return assembled(from * rigid_motion::matrix_exponential_so3(turned->first, s * turned->second), start + s * (waypoint().block<3, 1>(0, 3) - start));
}

// Modern Robotics eq. (9.8): a constant screw axis carrying the seed onto the waypoint.
transform screwed_at(double s)
{
    const auto logged = rigid_motion::matrix_logarithm_se3(rigid_motion::inverse(seed_frame()) * waypoint());
    REQUIRE(logged.has_value());
    return seed_frame() * rigid_motion::matrix_exponential_screw(logged->first, s * logged->second);
}

}

TEST_CASE("the_pose_waypoint_reference_holds_the_seed_and_the_waypoint_at_its_two_ends")
{
    const std::unique_ptr<pose_trajectory_generator> motion = commanded();

    REQUIRE(motion->duration() > 0.0);
    CHECK(is_approx_equal(at(*motion, 0.0).position, seed_frame(), 1.0e-9));
    CHECK(is_approx_equal(at(*motion, motion->duration()).position, waypoint(), 1.0e-9));
    CHECK(at(*motion, 0.0).velocity.isZero(1.0e-9));
    CHECK(at(*motion, motion->duration()).velocity.isZero(1.0e-9));
}

// Both right-hand sides are assembled from the published logarithm and exponential maps and the
// scaling's own answer. The second half says which of the two interpolations this reference is.
TEST_CASE("an_interior_sample_is_the_decoupled_interpolation_at_the_scaling_the_scalar_law_answers")
{
    const std::unique_ptr<pose_trajectory_generator> motion = commanded();
    const double span                                       = motion->duration();
    const double t                                          = 0.25 * span;
    const scaling_sample scaled                             = answered(quintic(t, span));

    CHECK(scaled.s > 0.0);
    CHECK(scaled.s < 1.0);
    CHECK(is_approx_equal(at(*motion, t).position, decoupled_at(scaled.s), 1.0e-9));
    CHECK_FALSE(is_approx_equal(at(*motion, t).position, screwed_at(scaled.s), 1.0e-6));
}

// The reported twists are compared against the time derivatives of the reported pose path itself, so
// an implementation dropping either term of the chain rule disagrees with the poses it reports.
TEST_CASE("the_reported_twists_are_the_time_derivatives_of_the_reported_pose_path")
{
    const std::unique_ptr<pose_trajectory_generator> motion = commanded();
    const double t                                          = 0.25 * motion->duration();

    const twist ahead  = coordinates_at(*motion, t + time_step);
    const twist here   = coordinates_at(*motion, t);
    const twist behind = coordinates_at(*motion, t - time_step);

    const twist velocity     = (ahead - behind) / (2.0 * time_step);
    const twist acceleration = (ahead - 2.0 * here + behind) / (time_step * time_step);

    CHECK((at(*motion, t).velocity - velocity).norm() < velocity_agreement);
    CHECK((at(*motion, t).acceleration - acceleration).norm() < acceleration_agreement);
}

// dxi/dt = xi'(s) s'(t) and d2xi/dt2 = xi''(s) s'(t)^2 + xi'(s) s''(t), the path's derivatives taken
// in s: Modern Robotics eq. (9.1) and (9.2). Both terms sit well above the agreement asserted at, so
// neither a dropped term nor a different scalar law passes this.
TEST_CASE("the_reported_derivatives_are_the_chain_rule_through_the_scalings_own_derivatives")
{
    const std::unique_ptr<pose_trajectory_generator> motion = commanded();
    const double span                                       = motion->duration();
    const double t                                          = 0.25 * span;
    const scaling_sample scaled                             = answered(quintic(t, span));

    const twist ahead  = coordinates(seed_frame(), answered(decoupled(seed_frame(), waypoint(), scaled.s + time_step)));
    const twist here   = coordinates(seed_frame(), answered(decoupled(seed_frame(), waypoint(), scaled.s)));
    const twist behind = coordinates(seed_frame(), answered(decoupled(seed_frame(), waypoint(), scaled.s - time_step)));

    const twist first         = (ahead - behind) / (2.0 * time_step);
    const twist second        = (ahead - 2.0 * here + behind) / (time_step * time_step);
    const pose_sample sampled = at(*motion, t);

    CHECK(std::abs(scaled.dds) > 0.0);
    CHECK((second * scaled.ds * scaled.ds).norm() > 100.0 * acceleration_agreement);
    CHECK((first * scaled.dds).norm() > 100.0 * acceleration_agreement);
    CHECK((sampled.velocity - first * scaled.ds).norm() < velocity_agreement);
    CHECK((sampled.acceleration - (second * scaled.ds * scaled.ds + first * scaled.dds)).norm() < acceleration_agreement);
}

// A pair takes a branch a longer run does not, so its duration, its pose and both its derivatives are
// held here against the published path, the published scaling and the pose path the motion itself
// reports -- none of them read off the factory.
TEST_CASE("a_pair_of_poses_is_swept_between_them_at_the_speed_the_wider_of_the_two_bounds_allows")
{
    const std::array<transform, 2> via{waypoint(), seed_frame()};
    auto held = decoupled_pose_waypoints(via, assembled(rotation::Identity(), Eigen::Vector3d{-4.0, -4.0, -4.0}), 0.5, 0.5);
    REQUIRE(held);

    const pose_trajectory_generator &motion = **held;
    const double span                       = motion.duration();
    const double t                          = 0.25 * span;
    const scaling_sample scaled             = answered(quintic(t, span));

    const twist ahead  = coordinates(via.front(), at(motion, t + time_step).position);
    const twist here   = coordinates(via.front(), at(motion, t).position);
    const twist behind = coordinates(via.front(), at(motion, t - time_step).position);

    CHECK(is_approx_equal(span, 2.0 * std::sqrt(3.5)));
    CHECK(is_approx_equal(at(motion, 0.0).position, via.front(), 1.0e-9));
    CHECK(is_approx_equal(at(motion, span).position, via.back(), 1.0e-9));
    CHECK(is_approx_equal(at(motion, t).position, answered(decoupled(via.front(), via.back(), scaled.s)), 1.0e-9));
    CHECK((at(motion, t).velocity - (ahead - behind) / (2.0 * time_step)).norm() < velocity_agreement);
    CHECK((at(motion, t).acceleration - (ahead - 2.0 * here + behind) / (time_step * time_step)).norm() < acceleration_agreement);
}

TEST_CASE("a_run_holding_no_pose_at_all_is_refused_rather_than_discarded")
{
    const auto empty = decoupled_pose_waypoints({}, seed_frame(), 0.5, 0.5);
    const auto swept = screw_pose_waypoints({}, seed_frame(), 0.5, 0.5);

    REQUIRE(!empty);
    CHECK(empty.error() == refusal::unsupported_input);
    REQUIRE(!swept);
    CHECK(swept.error() == refusal::unsupported_input);
}

TEST_CASE("a_traversal_that_takes_no_time_is_refused_rather_than_held_at_the_seed")
{
    const std::array<transform, 1> via{waypoint()};

    const auto unbounded  = decoupled_pose_waypoints(via, seed_frame(), 0.0, 0.0);
    const auto motionless = decoupled_pose_waypoints(std::array<transform, 1>{seed_frame()}, seed_frame(), 0.5, 0.5);

    REQUIRE(!unbounded);
    CHECK(unbounded.error() == refusal::unsupported_input);
    REQUIRE(!motionless);
    CHECK(motionless.error() == refusal::unsupported_input);
}
