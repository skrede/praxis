#include "seam_stage.h"
#include "substituted_trajectory.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/motion_commands.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <memory>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

transform planar_pose(double x, double y)
{
    transform pose = transform::Identity();
    pose(0, 3)     = x;
    pose(1, 3)     = y;

    return pose;
}

const std::array<joint_vector, 4> four_rows{configuration(0.3, 0.1), configuration(0.6, -0.2), configuration(0.2, 0.4), configuration(0.5, 0.5)};

const std::array<transform, 3> three_poses{planar_pose(0.3, 0.1), planar_pose(0.6, -0.2), planar_pose(0.2, 0.4)};

struct consumer_asks
{
    std::size_t run;
    std::size_t preview;
    std::size_t task_run;
};

// Each consumer is asked from where the one before it left the arm, and a run is played out before
// the next is asked, since a controller executing a motion takes no other.
consumer_asks asked_by_every_consumer(const trajectory::capabilities &shapes, std::size_t &(*counted)())
{
    const composed_arm arm = arm_under(shapes);
    consumer_asks asks{};

    reset_counts();
    arm.control->joint_space_trajectory(four_rows);
    asks.run = counted();
    REQUIRE(played_out(*arm.control));

    reset_counts();
    arm.control->preview_trajectory(std::span<const joint_vector>(four_rows));
    asks.preview = counted();

    reset_counts();
    arm.control->task_space_trajectory(three_poses);
    asks.task_run = counted();

    return asks;
}

bool same_schedule(std::span<const trajectory::via_point> left, std::span<const trajectory::via_point> right)
{
    if(left.size() != right.size())
        return false;

    for(std::size_t k = 0; k < left.size(); ++k)
        if(left[k].time != right[k].time || left[k].position != right[k].position || left[k].velocity != right[k].velocity)
            return false;

    return true;
}

}

TEST_CASE("a waypoint run is scheduled by the step the composition binds", "[seam][routing]")
{
    const composed_arm arm = arm_under(only_the_via_point_schedule());
    const std::array<joint_vector, 1> target{configuration(0.8, 0.8)};
    reset_counts();

    arm.control->joint_space_trajectory(target);

    REQUIRE(played_out(*arm.control));
    CHECK(is_approx_equal(arm.robot->joint_positions(), target.front(), 1.0e-9));
    CHECK(via_point_schedule_asks() > 0u);
}

TEST_CASE("every waypoint consumer runs the via-point slot the composition binds", "[seam][routing]")
{
    const consumer_asks asks = asked_by_every_consumer(only_the_via_point_slot(), &via_point_slot_asks);

    CHECK(asks.run > 0u);
    CHECK(asks.preview > 0u);
    CHECK(asks.task_run > 0u);
}

TEST_CASE("every waypoint consumer runs the step the composition binds", "[seam][routing]")
{
    const consumer_asks asks = asked_by_every_consumer(only_the_via_point_schedule(), &via_point_schedule_asks);

    CHECK(asks.run > 0u);
    CHECK(asks.preview > 0u);
    CHECK(asks.task_run > 0u);
}

TEST_CASE("a waypoint run joins exactly the via points the reference step chooses", "[seam][routing]")
{
    const composed_arm arm = arm_under(only_the_via_point_slot());
    const joint_vector j0  = arm.robot->joint_positions();

    arm.control->joint_space_trajectory(four_rows);

    const auto chosen = trajectory::baseline().trajectory.via_point_schedule(four_rows, j0, arm.robot->limits());
    REQUIRE(chosen);
    CHECK(same_schedule(last_joined(), *chosen));
}

// Each pose resolves to its own translation, so the configurations the reference resolves the three
// poses to are the rows the poses were written from.
TEST_CASE("a task-space waypoint run joins exactly the via points the reference step chooses for its resolved poses", "[seam][routing]")
{
    const composed_arm arm = arm_under(only_the_via_point_slot());
    const joint_vector j0  = arm.robot->joint_positions();
    const std::array<joint_vector, 3> resolved{four_rows[0], four_rows[1], four_rows[2]};

    arm.control->task_space_trajectory(three_poses);

    const auto chosen = trajectory::baseline().trajectory.via_point_schedule(resolved, j0, arm.robot->limits());
    REQUIRE(chosen);
    CHECK(same_schedule(last_joined(), *chosen));
}

TEST_CASE("a waypoint preview samples exactly the motion the reference step and slot give", "[seam][routing]")
{
    const composed_arm arm = arm_under(trajectory::baseline());
    const joint_vector j0  = arm.robot->joint_positions();

    arm.control->preview_trajectory(std::span<const joint_vector>(four_rows));

    const std::shared_ptr<const preview_run> run = arm.control->preview();
    const auto composed                          = via_point_motion(trajectory::baseline().trajectory, four_rows, j0, arm.robot->limits());
    REQUIRE(run != nullptr);
    REQUIRE(composed);
    CHECK(run->span == (*composed)->duration());
    for(const preview_sample &one : run->samples)
    {
        const auto expected_sample = (*composed)->sample(one.at);
        REQUIRE(expected_sample);
        CHECK(one.motion.position == expected_sample->position);
        CHECK(one.motion.velocity == expected_sample->velocity);
        CHECK(one.motion.acceleration == expected_sample->acceleration);
    }
}
