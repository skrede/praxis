#include "pose_run.h"
#include "captured_log.h"

#include "praxis/trajectory/baseline/pose_trajectory.h"

#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <numbers>
#include <utility>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::trajectory;

namespace {

using pose_run = expected<std::unique_ptr<pose_trajectory_generator>, refusal> (*)(std::span<const transform>, const transform &, double, double);

constexpr double linear_bound       = 0.5;
constexpr double angular_bound      = 0.5;
constexpr double pose_agreement     = 1.0e-9;
constexpr double duration_agreement = 1.0e-12;
constexpr double at_rest            = 1.0e-6;
constexpr double either_side        = 1.0e-4;

transform distant_seed()
{
    return assembled(rotation::Identity(), Eigen::Vector3d{-4.0, -4.0, -4.0});
}

std::vector<transform> four_poses()
{
    return {assembled(rotation::Identity(), Eigen::Vector3d{0.0, 0.0, 0.0}), turned_about(Eigen::Vector3d::UnitZ(), std::numbers::pi_v<double> / 2.0, Eigen::Vector3d{1.0, 0.5, 0.25}),
            turned_about(Eigen::Vector3d::UnitX(), std::numbers::pi_v<double> / 3.0, Eigen::Vector3d{0.5, 1.5, 0.75}),
            turned_about(Eigen::Vector3d::UnitY(), -std::numbers::pi_v<double> / 4.0, Eigen::Vector3d{2.0, 1.0, 0.5})};
}

// The traversal-time rule each segment is given, written out here rather than taken from the
// reference, so a change to either is a failure rather than a silent agreement.
double traversal(const transform &from, const transform &to)
{
    const auto turned = rigid_motion::matrix_logarithm_so3(rigid_motion::rotation_matrix_from_transform(from).transpose() * rigid_motion::rotation_matrix_from_transform(to));
    REQUIRE(turned.has_value());

    return std::max((to.block<3, 1>(0, 3) - from.block<3, 1>(0, 3)).norm() / linear_bound, std::abs(turned->second) / angular_bound);
}

std::vector<double> boundaries(std::span<const transform> poses)
{
    std::vector<double> times = {0.0};
    for(std::size_t k = 1u; k < poses.size(); ++k)
        times.push_back(times.back() + traversal(poses[k - 1u], poses[k]));

    return times;
}

std::unique_ptr<pose_trajectory_generator> carried_by(pose_run factory, std::span<const transform> poses)
{
    auto motion = factory(poses, distant_seed(), linear_bound, angular_bound);
    REQUIRE(motion);

    return std::move(*motion);
}

void check_stands_at(const transform &reached, const transform &pose)
{
    CHECK((reached.block<3, 1>(0, 3) - pose.block<3, 1>(0, 3)).norm() < pose_agreement);
    CHECK((reached.block<3, 3>(0, 0) - pose.block<3, 3>(0, 0)).norm() < pose_agreement);
}

}

TEST_CASE("a_run_of_four_poses_along_the_decoupled_path_lasts_the_three_traversals_it_is_made_of")
{
    const std::vector<transform> poses = four_poses();
    const auto motion                  = carried_by(&decoupled_pose_waypoints, poses);

    CHECK(is_approx_equal(motion->duration(), boundaries(poses).back(), duration_agreement));
}

TEST_CASE("a_run_of_four_poses_along_the_screw_path_lasts_the_three_traversals_it_is_made_of")
{
    const std::vector<transform> poses = four_poses();
    const auto motion                  = carried_by(&screw_pose_waypoints, poses);

    CHECK(is_approx_equal(motion->duration(), boundaries(poses).back(), duration_agreement));
}

TEST_CASE("a_run_of_four_poses_stands_at_each_of_them_at_its_own_boundary_along_either_path")
{
    const std::vector<transform> poses = four_poses();
    const std::vector<double> times    = boundaries(poses);

    for(pose_run factory : {&decoupled_pose_waypoints, &screw_pose_waypoints})
    {
        const auto motion = carried_by(factory, poses);
        for(std::size_t k = 0u; k < poses.size(); ++k)
            check_stands_at(at(*motion, times[k]).position, poses[k]);
    }
}

// Each segment carries its own scaling, whose rate is zero at both of its ends, so the motion comes
// fully to rest at every pose between the two it runs between rather than passing it at speed.
TEST_CASE("a_run_of_four_poses_comes_to_rest_at_every_pose_it_is_carried_through")
{
    const std::vector<transform> poses = four_poses();
    const std::vector<double> times    = boundaries(poses);
    const auto motion                  = carried_by(&decoupled_pose_waypoints, poses);

    CHECK(at(*motion, 0.0).velocity.norm() < at_rest);
    CHECK(at(*motion, motion->duration()).velocity.norm() < at_rest);

    for(std::size_t k = 1u; k + 1u < times.size(); ++k)
    {
        CHECK(at(*motion, times[k] - either_side).velocity.norm() < at_rest);
        CHECK(at(*motion, times[k]).velocity.norm() < at_rest);
        CHECK(at(*motion, times[k] + either_side).velocity.norm() < at_rest);
    }
}

TEST_CASE("a_run_of_three_poses_begins_at_the_first_of_them_and_not_at_the_seed")
{
    const std::vector<transform> all = four_poses();
    const std::vector<transform> poses(all.begin(), all.begin() + 3);
    const auto motion = carried_by(&decoupled_pose_waypoints, poses);

    check_stands_at(at(*motion, 0.0).position, poses.front());
    check_stands_at(at(*motion, motion->duration()).position, poses.back());
    CHECK_FALSE(is_approx_equal(at(*motion, 0.0).position, distant_seed(), 1.0e-6));
}

TEST_CASE("a_sample_past_the_end_of_a_run_of_poses_answers_the_pose_it_ends_at")
{
    const std::vector<transform> poses = four_poses();
    const auto motion                  = carried_by(&decoupled_pose_waypoints, poses);

    check_stands_at(at(*motion, 10.0 * motion->duration()).position, poses.back());
}

TEST_CASE("a_run_repeating_the_pose_it_already_stands_at_is_refused_and_the_pair_is_named")
{
    const std::vector<transform> all   = four_poses();
    const std::vector<transform> poses = {all[0], all[1], all[1], all[2]};

    praxis::tests::captured_log captured;
    const auto refused         = decoupled_pose_waypoints(poses, distant_seed(), linear_bound, angular_bound);
    const std::string reported = captured.text();

    REQUIRE(!refused);
    CHECK(refused.error() == refusal::unsupported_input);
    CHECK(reported.find("row 2") != std::string::npos);
}
