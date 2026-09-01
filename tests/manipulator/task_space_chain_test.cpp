#include "captured_log.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/motion.h"
#include "praxis/manipulator/baseline/kinematics.h"
#include "praxis/manipulator/baseline/task_trajectory.h"

#include "praxis/evaluation/tolerance.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <cmath>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

constexpr double upper_arm = 0.5;
constexpr double forearm   = 0.4;
constexpr double reaches   = 1.0e-6;

joint_vector configuration(double first, double second)
{
    joint_vector q(2);
    q << first, second;

    return q;
}

// A planar two-link arm: both joints rotate about the world z, the second sits at the far end of the
// first link, and the tool frame is the far end of the second. Lynch & Park, Modern Robotics,
// example 4.3 -- a screw axis of a revolute joint is (w, -w x q) for a point q on the axis.
screw_chain planar_arm()
{
    screw_axis shoulder;
    shoulder << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    screw_axis elbow;
    elbow << 0.0, 0.0, 1.0, 0.0, -upper_arm, 0.0;

    transform home = transform::Identity();
    home(0, 3)     = upper_arm + forearm;

    joint_limits bounds{};
    bounds.velocity       = configuration(1.0, 0.5);
    bounds.acceleration   = configuration(2.0, 2.0);
    bounds.lower_position = configuration(-3.0, -3.0);
    bounds.upper_position = configuration(3.0, 3.0);

    return screw_chain(home, {shoulder, elbow}, bounds);
}

kinematics reference()
{
    auto solver = manipulator::make_kinematics(planar_arm(), manipulator::baseline().fk, manipulator::baseline().dk, manipulator::baseline().ik, rigid_motion::baseline().screw,
                                               rigid_motion::baseline().frame);
    REQUIRE(solver);

    return std::move(*solver);
}

transform reached(const kinematics &solver, const joint_vector &q)
{
    const expected<transform, refusal> pose = solver.fk_solve(q);
    REQUIRE(pose);

    return *pose;
}

std::vector<transform> four_poses(const kinematics &solver)
{
    return {reached(solver, configuration(0.2, 0.3)), reached(solver, configuration(0.6, -0.4)), reached(solver, configuration(-0.3, 0.9)), reached(solver, configuration(0.9, 0.5))};
}

// The chain of resolutions the factory makes, re-derived through the same shipped motion capability,
// so the configurations the run is asserted against are not read back out of the run itself.
std::vector<joint_vector> resolved(const kinematics &solver, std::span<const transform> poses, const joint_vector &j0)
{
    std::vector<joint_vector> chain;
    joint_vector seed = j0;

    for(const transform &pose : poses)
    {
        const expected<joint_vector, refusal> answer = manipulator::task_space_pose(solver, pose, seed);
        REQUIRE(answer);
        seed = *answer;
        chain.push_back(seed);
    }

    return chain;
}

// The shares the knot times stand in proportion to, written out here rather than taken from the
// reference. One factor stretches the whole run, so carrying the shares onto its duration is exact.
std::vector<double> boundaries(std::span<const joint_vector> chain, const joint_vector &velocity, double span)
{
    std::vector<double> times = {0.0};

    for(std::size_t k = 1u; k < chain.size(); ++k)
    {
        double share = 0.0;
        for(Eigen::Index i = 0; i < chain[k].size(); ++i)
            share = std::max(share, std::abs(chain[k][i] - chain[k - 1u][i]) / velocity[i]);

        times.push_back(times.back() + share);
    }

    for(double &knot : times)
        knot *= span / times.back();

    return times;
}

trajectory::trajectory_sample at(const trajectory::trajectory_generator &motion, double t)
{
    auto sampled = motion.sample(t);
    REQUIRE(sampled);

    return *sampled;
}

}

TEST_CASE("a_chain_of_four_poses_is_traversed_where_the_count_was_refused")
{
    const kinematics solver            = reference();
    const std::vector<transform> poses = four_poses(solver);
    const auto motion                  = manipulator::task_space_waypoints(solver, poses, configuration(0.0, 0.0), planar_arm().limits);

    REQUIRE(motion.has_value());
    CHECK((*motion)->duration() > 0.0);
}

// Carrying the sampled configuration back through the arm's own forward map is what makes this a
// chain of poses rather than a chain of configurations that happens to begin at the right one.
TEST_CASE("each_knot_of_a_chain_reaches_the_pose_it_was_given_through_the_arms_own_forward_map")
{
    const kinematics solver            = reference();
    const std::vector<transform> poses = four_poses(solver);
    const joint_vector seed            = configuration(0.0, 0.0);
    const joint_limits limits          = planar_arm().limits;
    const auto motion                  = manipulator::task_space_waypoints(solver, poses, seed, limits);
    REQUIRE(motion.has_value());

    const std::vector<double> times = boundaries(resolved(solver, poses, seed), limits.velocity, (*motion)->duration());

    for(std::size_t k = 0u; k < poses.size(); ++k)
    {
        const transform standing = reached(solver, at(**motion, times[k]).position);

        CHECK((standing.block<3, 1>(0, 3) - poses[k].block<3, 1>(0, 3)).norm() < reaches);
        CHECK((standing.block<3, 3>(0, 0) - poses[k].block<3, 3>(0, 0)).norm() < reaches);
    }
}

TEST_CASE("a_chain_holding_a_pose_beyond_the_arms_reach_is_refused_and_the_waypoint_is_named")
{
    const kinematics solver          = reference();
    const std::vector<transform> all = four_poses(solver);
    transform beyond                 = transform::Identity();
    beyond(0, 3)                     = 10.0;

    praxis::tests::captured_log captured;
    const auto refused         = manipulator::task_space_waypoints(solver, std::vector<transform>{all[0], all[1], beyond, all[2]}, configuration(0.0, 0.0), planar_arm().limits);
    const std::string reported = captured.text();

    REQUIRE_FALSE(refused.has_value());
    CHECK(reported.find("waypoint 3") != std::string::npos);
}
