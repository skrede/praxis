#include "answered.h"

#include "praxis/trajectory/baseline/time_scaling.h"
#include "praxis/trajectory/baseline/pose_trajectory.h"

#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <array>
#include <memory>
#include <numbers>
#include <utility>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::trajectory;

namespace {

using pose_waypoint_factory = decltype(pose_trajectory_ops::screw_pose_waypoints);

constexpr double time_step              = 1.0e-4;
constexpr double velocity_agreement     = 1.0e-7;
constexpr double acceleration_agreement = 1.0e-5;

transform assembled(const rotation &r, const Eigen::Vector3d &p)
{
    transform tf         = transform::Identity();
    tf.block<3, 3>(0, 0) = r;
    tf.block<3, 1>(0, 3) = p;
    return tf;
}

transform seed_frame()
{
    return assembled(rotation::Identity(), Eigen::Vector3d{1.0, 0.0, 0.25});
}

// A quarter turn about the world z with a translation that is not along that axis, which is the
// family the two task-space paths separate on: a translation along the axis of the turn would leave
// them agreeing.
transform waypoint()
{
    return assembled(Eigen::AngleAxisd(std::numbers::pi_v<double> / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix(), Eigen::Vector3d{0.0, 1.5, 0.75});
}

// The exponential coordinates of a pose relative to the seed: the curve the reported twist and its
// derivative are derivatives of.
twist coordinates(const transform &pose)
{
    const expected<std::pair<screw_axis, double>, refusal> logged = rigid_motion::matrix_logarithm_se3(rigid_motion::inverse(seed_frame()) * pose);

    REQUIRE(logged.has_value());
    return logged->first * logged->second;
}

pose_sample at(const pose_trajectory_generator &motion, double t)
{
    auto sampled = motion.sample(t);

    REQUIRE(sampled);
    return *sampled;
}

// Modern Robotics eq. (9.8): a constant screw axis carrying the seed onto the waypoint.
transform screwed_at(double s)
{
    const auto logged = rigid_motion::matrix_logarithm_se3(rigid_motion::inverse(seed_frame()) * waypoint());

    REQUIRE(logged.has_value());
    return seed_frame() * rigid_motion::matrix_exponential_screw(logged->first, s * logged->second);
}

std::unique_ptr<pose_trajectory_generator> commanded(pose_waypoint_factory along)
{
    const std::array<transform, 1> via{waypoint()};

    auto motion = along(via, seed_frame(), 0.5, 0.5);

    REQUIRE(motion);
    return std::move(*motion);
}

}

TEST_CASE("an_interior_sample_of_the_screw_slot_is_the_exponential_of_the_scaled_logarithm_of_the_relative_motion")
{
    const std::unique_ptr<pose_trajectory_generator> screwed = commanded(&screw_pose_waypoints);
    const std::unique_ptr<pose_trajectory_generator> chorded = commanded(&decoupled_pose_waypoints);
    const double t                                           = 0.25 * screwed->duration();
    const scaling_sample scaled                              = answered(quintic(t, screwed->duration()));

    CHECK(scaled.s > 0.0);
    CHECK(scaled.s < 1.0);
    CHECK(is_approx_equal(at(*screwed, t).position, screwed_at(scaled.s), 1.0e-9));
    CHECK_FALSE(is_approx_equal(at(*chorded, t).position, screwed_at(scaled.s), 1.0e-6));
}

// The two slots share their ends, so a disagreement between them can only be the path taken between.
TEST_CASE("the_two_pose_waypoint_slots_carry_the_same_ends_along_different_paths")
{
    const std::unique_ptr<pose_trajectory_generator> screwed = commanded(&screw_pose_waypoints);
    const std::unique_ptr<pose_trajectory_generator> chorded = commanded(&decoupled_pose_waypoints);
    const double span                                        = screwed->duration();

    REQUIRE(is_approx_equal(span, chorded->duration()));
    CHECK(is_approx_equal(at(*screwed, 0.0).position, at(*chorded, 0.0).position, 1.0e-9));
    CHECK(is_approx_equal(at(*screwed, span).position, at(*chorded, span).position, 1.0e-9));

    for(double fraction : std::array<double, 3>{0.25, 0.5, 0.75})
        CHECK_FALSE(is_approx_equal(at(*screwed, fraction * span).position, at(*chorded, fraction * span).position, 1.0e-6));
}

// The chain rule through the scaling's own derivatives reaches this slot too: the reported twists
// are the time derivatives of the poses this slot itself reports.
TEST_CASE("the_screw_slots_reported_twists_are_the_time_derivatives_of_its_own_pose_path")
{
    const std::unique_ptr<pose_trajectory_generator> screwed = commanded(&screw_pose_waypoints);
    const double t                                           = 0.25 * screwed->duration();

    const twist ahead  = coordinates(at(*screwed, t + time_step).position);
    const twist here   = coordinates(at(*screwed, t).position);
    const twist behind = coordinates(at(*screwed, t - time_step).position);

    CHECK((at(*screwed, t).velocity - (ahead - behind) / (2.0 * time_step)).norm() < velocity_agreement);
    CHECK((at(*screwed, t).acceleration - (ahead - 2.0 * here + behind) / (time_step * time_step)).norm() < acceleration_agreement);
}
