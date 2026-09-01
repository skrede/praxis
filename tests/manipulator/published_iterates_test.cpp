#include "fixtures.h"
#include "captured_log.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <span>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;
using namespace praxis::scheduler;

namespace {

time_point dictated{};

time_point reading()
{
    return dictated;
}

clock_source dictating()
{
    dictated = time_point{};
    return clock_source{&reading};
}

// One service covers every period the raised reading spans, so the advance may be coarser than the
// period the playback is registered at.
constexpr seconds serviced{0.05};

// The answer is the desired pose's own translation, so a command reaches a configuration the case
// can name, and the one recorded step is identifiable by its index alone.
praxis::expected<void, praxis::refusal> recording_inverse_kinematics(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &,
                                                                     const praxis::transform &desired, const joint_vector &j0, const solver_parameters &, ik_result &answer)
{
    answer.iterations.push_back(iteration_state{j0, 0.5, 0.25, 0.125, 7u});
    answer.solutions.push_back(configuration(desired(0, 3), desired(1, 3)));

    return {};
}

// The step is reported and no solution is offered, so the solve is genuinely entered and ik_solve
// refuses afterwards on an empty solution set rather than before the solver is reached.
praxis::expected<void, praxis::refusal> fruitless_inverse_kinematics(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const praxis::transform &,
                                                                     const joint_vector &j0, const solver_parameters &, ik_result &answer)
{
    answer.iterations.push_back(iteration_state{j0, 0.5, 0.25, 0.125, 7u});

    return {};
}

praxis::expected<joint_vector, praxis::refusal> solving_task_space_pose(const kinematics &solver, const praxis::transform &pose, const joint_vector &j0)
{
    return solver.ik_solve(pose, j0, solver_parameters{});
}

praxis::transform reachable(double x, double y)
{
    praxis::transform pose = praxis::transform::Identity();
    pose(0, 3)             = x;
    pose(1, 3)             = y;

    return pose;
}

forward_kinematics_ops sliding()
{
    return forward_kinematics_ops{.forward_kinematics = &sliding_forward_kinematics};
}

inverse_kinematics_ops solving()
{
    return inverse_kinematics_ops{&recording_inverse_kinematics};
}

inverse_kinematics_ops solving_but_finding_nothing()
{
    return inverse_kinematics_ops{&fruitless_inverse_kinematics};
}

// The robot and the controller alone, with none of the strand-owned state around them, so a case can
// drive the controller directly and see what an extent bounds when nothing opens one.
struct bare_arm
{
    std::shared_ptr<scene_robot> robot;
    std::unique_ptr<robot_controller> control;
};

// The seeded configuration is the one every case starts from, over a composition that binds no
// tool-frame displacement, so a jog is a motion operation that refuses before the solver.
bare_arm compose_arm(const forward_kinematics_ops &forward, const differential_kinematics_ops &differential, const inverse_kinematics_ops &inverse)
{
    auto robot = std::make_shared<scene_robot>(
            scene_robot::compose(
                    kinematics::compose(sliding_chain(), forward, differential, inverse, praxis::rigid_motion::baseline().screw, praxis::rigid_motion::baseline().frame).value(),
                    robot_ops{}, praxis::rigid_motion::baseline().frame, 2u)
                    .value());
    robot->set_joint_positions(configuration(0.25, -0.5));

    auto control = std::make_unique<robot_controller>(*robot, motion_ops{.task_space_pose = &solving_task_space_pose}, composing_path(), task_trajectory_ops{}, composing_time_scaling(),
                                                      praxis::trajectory::trajectory_ops{}, praxis::rigid_motion::screw_ops{});

    return bare_arm{std::move(robot), std::move(control)};
}

// What a case reaches the pipe through: the robot, the controller and the publisher are reachable
// from nowhere but the aggregate the gate owns.
struct arm_pipe
{
    std::shared_ptr<owned_arm> owned;
    arm_reader seen;
};

arm_pipe pipe(scheduler &loop, const forward_kinematics_ops &forward = sliding(), const differential_kinematics_ops &differential = differential_kinematics_ops{},
              const inverse_kinematics_ops &inverse = solving())
{
    const strand work = *loop.make_strand();
    bare_arm bare     = compose_arm(forward, differential, inverse);

    std::shared_ptr<robot_controller> controller = std::move(bare.control);
    auto published                               = std::make_shared<arm_publisher>();

    return arm_pipe{std::make_shared<owned_arm>(work, work, bare.robot, controller, published), published->reader()};
}

}

TEST_CASE("a command whose motion slot refused before the solver publishes no sequence and carries none of the command before it", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm                      = pipe(loop);
    const std::weak_ptr<owned_arm> observer = arm.owned;

    command(observer, [](robot_controller &control, scene_robot &) { control.preview_task_space_pose(reachable(0.5, -0.25)); });
    REQUIRE(loop.drain().has_value());
    REQUIRE(arm.seen.read()->iterations.size() == 1);

    const std::string reported = reported_by(
            [&loop, &observer]
            {
                command(observer, [](robot_controller &control, scene_robot &)
                        { control.preview_tool_frame_jog(praxis::transform::Identity(), Eigen::Vector3d::Zero(), praxis::rotation::Identity()); });
                REQUIRE(loop.drain().has_value());
            });

    // The collection is what this command asked the solver for, and it asked for nothing: the
    // sequence the command before it left behind belongs to that command and not to this one.
    CHECK(arm.seen.read()->iterations.empty());
    CHECK(reported.find("motion.tool_frame_displace") != std::string::npos);
}

// A solve that converged on nothing is still a solve: its iterates are exactly what a reader has to
// look at to see why no answer was reached, so the collection is about entries into the solver and
// not about answers that came back out.
TEST_CASE("a solve that ran and found nothing still publishes the iterates it reported on the way", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm = pipe(loop, sliding(), differential_kinematics_ops{}, solving_but_finding_nothing());

    const std::string reported = reported_by(
            [&loop, &arm]
            {
                command(std::weak_ptr<owned_arm>(arm.owned), [](robot_controller &control, scene_robot &) { control.preview_task_space_pose(reachable(0.5, -0.25)); });
                REQUIRE(loop.drain().has_value());
            });

    const std::shared_ptr<const arm_snapshot> seen = arm.seen.read();

    REQUIRE(seen->iterations.size() == 1);
    REQUIRE(seen->iterations.front().size() == 1);
    CHECK(seen->iterations.front().front().index == 7u);
    CHECK(reported.find("motion.task_space_pose") != std::string::npos);

    // Every value commanded here is one single precision carries exactly, so a substituted value
    // cannot hide inside a tolerance.
    REQUIRE(seen->joints.size() == 2);
    CHECK(seen->joints[0] == 0.25);
    CHECK(seen->joints[1] == -0.5);
}

TEST_CASE("a playback advance is the same command still running and leaves its sequences standing", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm = pipe(loop);

    command(std::weak_ptr<owned_arm>(arm.owned), [](robot_controller &control, scene_robot &) { control.task_space_ptp(reachable(0.5, -0.25)); });
    REQUIRE(loop.drain().has_value());

    const std::shared_ptr<const arm_snapshot> started = arm.seen.read();

    REQUIRE(started->iterations.size() == 1);
    REQUIRE(started->executing);

    dictated += std::chrono::duration_cast<time_point::duration>(serviced);
    REQUIRE(loop.drain().has_value());

    const std::shared_ptr<const arm_snapshot> advanced = arm.seen.read();

    // A publication of its own is what proves the periodic task fired: without it the survival below
    // would hold over the publication the command itself left standing.
    REQUIRE(advanced != started);
    REQUIRE(advanced->iterations.size() == 1);
    REQUIRE(advanced->iterations.front().size() == 1);
    CHECK(advanced->iterations.front().front().index == 7u);
}

TEST_CASE("a command that hands the solver nothing leaves a motion in flight its sequences", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm                      = pipe(loop);
    const std::weak_ptr<owned_arm> observer = arm.owned;

    command(observer, [](robot_controller &control, scene_robot &) { control.task_space_ptp(reachable(0.5, -0.25)); });
    REQUIRE(loop.drain().has_value());

    const std::shared_ptr<const arm_snapshot> started = arm.seen.read();

    REQUIRE(started->iterations.size() == 1);
    REQUIRE(started->executing);

    command(observer, [](robot_controller &control, scene_robot &) { control.set_velocity_factor(0.5); });
    REQUIRE(loop.drain().has_value());

    const std::shared_ptr<const arm_snapshot> nudged = arm.seen.read();

    // A publication of its own is what proves the second command ran: without it the survival below
    // would hold over the publication the first command left standing.
    REQUIRE(nudged != started);
    CHECK(nudged->executing);
    REQUIRE(nudged->iterations.size() == 1);
    REQUIRE(nudged->iterations.front().size() == 1);
    CHECK(nudged->iterations.front().front().index == 7u);
    CHECK(nudged->velocity_factor == 0.5);
}

TEST_CASE("a controller driven with no command open keeps one request's solves at most", "[manipulator][ownership]")
{
    const bare_arm arm = compose_arm(sliding(), differential_kinematics_ops{}, solving());

    arm.control->preview_task_space_pose(reachable(0.5, -0.25));
    REQUIRE(arm.control->solves().size() == 1);

    arm.control->preview_task_space_pose(reachable(0.75, 0.5));
    REQUIRE(arm.control->solves().size() == 1);
}

// Each sequence carries the configuration its own solve started from, so the two are told apart by
// what they hold rather than by their order alone.
TEST_CASE("two requests made inside one command are both recoverable in the order they were made", "[manipulator][ownership]")
{
    const bare_arm arm = compose_arm(sliding(), differential_kinematics_ops{}, solving());
    const robot_controller::command_extent one(*arm.control);

    arm.control->preview_task_space_pose(reachable(0.5, -0.25));
    arm.control->preview_task_space_pose(reachable(0.75, 0.5));

    const std::span<const std::vector<iteration_state>> made = arm.control->solves();

    REQUIRE(made.size() == 2);
    REQUIRE(made[0].size() == 1);
    REQUIRE(made[1].size() == 1);
    CHECK(made[0].front().joint_positions[0] == 0.25);
    CHECK(made[1].front().joint_positions[0] == 0.5);
}
