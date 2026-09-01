#include "praxis/manipulator/baseline/task_trajectory.h"

#include "praxis/evaluation/tolerance.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <memory>
#include <vector>
#include <cstdint>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

joint_vector configuration(double first, double second)
{
    joint_vector q(2);
    q << first, second;

    return q;
}

joint_limits bounds()
{
    joint_limits limits{};
    limits.velocity     = configuration(1.0, 0.5);
    limits.acceleration = configuration(2.0, 2.0);

    return limits;
}

transform at_height(double z)
{
    transform pose = transform::Identity();
    pose(2, 3)     = z;

    return pose;
}

// The resolution is a pure function of the pose and the seed, so a chain of waypoints resolved from
// each other's answers is observable in the result, and the seeds are recorded besides. A slot is a
// plain function, so the record it keeps is file-local rather than a member of a solver object.
std::vector<joint_vector> recorded_seeds;

expected<void, refusal> recording_inverse_kinematics(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const transform &desired_pose,
                                                     const joint_vector &j0, const solver_parameters &, ik_result &answer)
{
    recorded_seeds.push_back(j0);
    answer.solutions.push_back(j0 + joint_vector::Constant(j0.size(), desired_pose(2, 3)));

    return {};
}

trajectory::trajectory_sample at(const trajectory::trajectory_generator &motion, double t)
{
    auto sampled = motion.sample(t);
    REQUIRE(sampled);

    return *sampled;
}

// Composing reads the limits and nothing else off the chain, so they carry one entry per joint.
screw_chain two_joint_chain()
{
    joint_limits limits   = bounds();
    limits.lower_position = configuration(-3.0, -3.0);
    limits.upper_position = configuration(3.0, 3.0);

    return screw_chain(transform::Identity(), {screw_axis::Zero(), screw_axis::Zero()}, limits);
}

kinematics recording_solver()
{
    recorded_seeds.clear();

    return kinematics::compose(two_joint_chain(), {}, {}, inverse_kinematics_ops{.inverse_kinematics = &recording_inverse_kinematics}, rigid_motion::baseline().screw,
                               rigid_motion::baseline().frame)
            .value();
}

}

TEST_CASE("a_single_task_space_waypoint_runs_from_the_seed_to_the_pose_it_resolves_to")
{
    const kinematics solver          = recording_solver();
    const joint_vector seed          = configuration(0.0, 0.0);
    const std::vector<transform> via = {at_height(0.5)};
    const auto motion                = manipulator::task_space_waypoints(solver, via, seed, bounds());

    REQUIRE(motion.has_value());
    REQUIRE(recorded_seeds.size() == 1u);
    CHECK(is_approx_equal(recorded_seeds.front(), seed));
    CHECK(is_approx_equal(at(**motion, 0.0).position, seed));
    CHECK(is_approx_equal(at(**motion, (*motion)->duration()).position, configuration(0.5, 0.5)));
}

TEST_CASE("a_chain_of_task_space_waypoints_seeds_each_resolution_from_the_previous_answer")
{
    const kinematics solver          = recording_solver();
    const joint_vector seed          = configuration(0.1, 0.1);
    const std::vector<transform> via = {at_height(0.5), at_height(0.25), at_height(0.75), at_height(0.3)};
    const auto motion                = manipulator::task_space_waypoints(solver, via, seed, bounds());

    REQUIRE(motion.has_value());
    REQUIRE(recorded_seeds.size() == 4u);
    CHECK(is_approx_equal(recorded_seeds.front(), seed));
    CHECK(is_approx_equal(recorded_seeds[1], configuration(0.6, 0.6)));
    CHECK(is_approx_equal(recorded_seeds[2], configuration(0.85, 0.85)));
    CHECK(is_approx_equal(recorded_seeds.back(), configuration(1.6, 1.6)));
    CHECK(is_approx_equal(at(**motion, 0.0).position, configuration(0.6, 0.6)));
    CHECK(is_approx_equal(at(**motion, (*motion)->duration()).position, configuration(1.9, 1.9)));
}

TEST_CASE("a_chain_of_three_poses_is_traversed_through_the_configurations_they_resolve_to")
{
    const kinematics solver            = recording_solver();
    const joint_vector seed            = configuration(0.3, -0.2);
    const std::vector<transform> poses = {at_height(0.5), at_height(0.25), at_height(0.75)};
    const auto motion                  = manipulator::task_space_waypoints(solver, poses, seed, bounds());

    REQUIRE(motion.has_value());
    REQUIRE(recorded_seeds.size() == 3u);
    CHECK(is_approx_equal(at(**motion, 0.0).position, configuration(0.8, 0.3)));
    CHECK(is_approx_equal(at(**motion, (*motion)->duration()).position, configuration(1.8, 1.3)));
}

TEST_CASE("a_via_point_request_carrying_no_pose_is_refused_before_anything_is_resolved")
{
    const kinematics solver = recording_solver();
    const auto motion       = manipulator::task_space_waypoints(solver, {}, configuration(0.1, 0.1), bounds());

    REQUIRE_FALSE(motion.has_value());
    CHECK(motion.error() == refusal::unsupported_input);
    CHECK(recorded_seeds.empty());
}

// The factory resolves each pose through the motion capability, so a resolution it cannot make is
// the factory's own refusal and no partial motion is assembled from the poses that did resolve.
TEST_CASE("a_pose_that_resolves_to_nothing_refuses_the_whole_via_point_motion")
{
    const kinematics solver            = recording_solver();
    const joint_vector seed            = joint_vector::Constant(3, 0.2);
    const std::vector<transform> poses = {at_height(0.5), at_height(0.25)};
    const auto motion                  = manipulator::task_space_waypoints(solver, poses, seed, bounds());

    REQUIRE_FALSE(motion.has_value());
    CHECK(motion.error() == refusal::unsupported_input);
    CHECK(recorded_seeds.empty());
}

TEST_CASE("the_via_point_factory_slot_defaults_to_refusing_rather_than_to_a_motion_that_holds_the_seed")
{
    const task_trajectory_ops ops{};
    const kinematics solver = recording_solver();
    const auto motion       = ops.task_space_waypoints(solver, {}, configuration(0.1, 0.1), bounds());

    REQUIRE_FALSE(motion.has_value());
    CHECK(motion.error() == refusal::not_implemented);
}
