#include "fixtures.h"

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/evaluation.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/tolerance.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>

using namespace praxis;
using namespace praxis::evaluation;
using namespace praxis::manipulator;

namespace {

constexpr std::size_t seeds_walked = 40u;

using task_waypoint_slot = expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> (*)(const trajectory::trajectory_ops &, const motion_ops &, const kinematics &,
                                                                                                    std::span<const transform>, const joint_vector &, const joint_limits &);

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> joined(const trajectory::trajectory_ops &trajectory, std::span<const joint_vector> rows, const joint_vector &j0,
                                                                            const joint_limits &limits)
{
    const expected<std::vector<trajectory::via_point>, refusal> schedule = trajectory.via_point_schedule(rows, j0, limits);
    if(!schedule)
        return praxis::unexpected(schedule.error());

    return trajectory.joint_space_via_points(*schedule);
}

// Each pose is resolved from the previous resolution, a lone pose joined to it from j0.
expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> seeded_forward(const trajectory::trajectory_ops &trajectory, const motion_ops &motion, const kinematics &solver,
                                                                                    std::span<const transform> waypoints, const joint_vector &j0, const joint_limits &limits)
{
    std::vector<joint_vector> rows;
    if(waypoints.size() == 1u)
        rows.push_back(j0);

    joint_vector seed = j0;
    for(const transform &pose : waypoints)
    {
        const expected<joint_vector, refusal> reached = motion.task_space_pose(solver, pose, seed);
        if(!reached)
            return praxis::unexpected(reached.error());

        seed = *reached;
        rows.push_back(seed);
    }

    return joined(trajectory, rows, j0, limits);
}

// The rows start at j0 and each solve is seeded from the last row; j0 is dropped again when more
// than one pose was given.
expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> seeded_from_last_row(const trajectory::trajectory_ops &trajectory, const motion_ops &motion,
                                                                                          const kinematics &solver, std::span<const transform> waypoints, const joint_vector &j0,
                                                                                          const joint_limits &limits)
{
    std::vector<joint_vector> rows{j0};
    for(std::size_t i = 0; i < waypoints.size(); ++i)
    {
        const expected<joint_vector, refusal> reached = motion.task_space_pose(solver, waypoints[i], rows.back());
        if(!reached)
            return praxis::unexpected(reached.error());

        rows.push_back(*reached);
    }
    if(waypoints.size() > 1u)
        rows.erase(rows.begin());

    return joined(trajectory, rows, j0, limits);
}

constexpr std::array<task_waypoint_slot, 2> literal_slots{&seeded_forward, &seeded_from_last_row};

std::vector<joint_vector> seeds_handed;

expected<joint_vector, refusal> counted_translation(const kinematics &, const transform &pose, const joint_vector &seed)
{
    seeds_handed.push_back(seed);

    return fixture::configuration(pose(0, 3), pose(1, 3));
}

transform planar_pose(double x, double y)
{
    transform pose = transform::Identity();
    pose(0, 3)     = x;
    pose(1, 3)     = y;

    return pose;
}

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> counted_run(std::span<const transform> poses, const joint_vector &j0)
{
    const kinematics solver = fixture::sliding_solver();
    seeds_handed.clear();

    return manipulator::baseline().trajectory.task_space_waypoints(trajectory::baseline().trajectory, motion_ops{.task_space_pose = &counted_translation}, solver, poses, j0,
                                                                   solver.space_chain().limits);
}

// The task trajectory view is the last the aggregate lists.
slot_report judged(task_waypoint_slot slot, std::uint64_t seed)
{
    const capabilities reference            = manipulator::baseline();
    capabilities literal                    = manipulator::baseline();
    literal.trajectory.task_space_waypoints = slot;
    const auto views                        = evaluation_views(reference, literal);

    return evaluate(std::span(views).last(1), seed, default_cases_per_slot).slots.at(0);
}

}

TEST_CASE("a_literal_task_space_waypoint_run_in_each_arrangement_is_judged_agreeing_by_the_task_space_waypoint_row_at_each_of_forty_seeds")
{
    for(std::size_t offset = 0; offset < seeds_walked; ++offset)
        for(std::size_t form = 0; form < literal_slots.size(); ++form)
        {
            const slot_report row = judged(literal_slots.at(form), default_seed + offset);

            INFO("seed offset " << offset << ", form " << form);
            CHECK(row.slot == "trajectory.task_space_waypoints");
            CHECK(row.verdict == agreement::agreed);
            CHECK(row.outcomes.differed == 0u);
            CHECK(row.outcomes.agreed + row.outcomes.both_refused == default_cases_per_slot);
        }
}

TEST_CASE("a_task_space_waypoint_run_resolves_every_pose_through_the_motion_operations_it_is_handed_each_seeded_by_the_pose_before")
{
    const joint_vector j0 = fixture::configuration(0.1, 0.2);
    const std::array<transform, 3> poses{planar_pose(0.3, 0.1), planar_pose(0.6, -0.2), planar_pose(0.2, 0.4)};

    const auto run = counted_run(poses, j0);

    REQUIRE(run);
    REQUIRE(seeds_handed.size() == 3u);
    CHECK(seeds_handed[0] == j0);
    CHECK(seeds_handed[1] == fixture::configuration(0.3, 0.1));
    CHECK(seeds_handed[2] == fixture::configuration(0.6, -0.2));
}

TEST_CASE("a_task_space_waypoint_run_of_one_pose_is_joined_to_it_from_j0")
{
    const joint_vector j0 = fixture::configuration(0.1, 0.2);
    const std::array<transform, 1> pose{planar_pose(0.6, -0.2)};

    const auto run = counted_run(pose, j0);

    REQUIRE(run);
    REQUIRE(seeds_handed.size() == 1u);
    const auto start = (*run)->sample(0.0);
    const auto end   = (*run)->sample((*run)->duration());
    REQUIRE(start);
    REQUIRE(end);
    CHECK(start->position == j0);
    CHECK(is_approx_equal(end->position, fixture::configuration(0.6, -0.2), 1.0e-12));
}
