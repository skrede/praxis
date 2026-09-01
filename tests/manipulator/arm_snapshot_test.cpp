#include "fixtures.h"
#include "captured_log.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <memory>
#include <string>
#include <cstdint>

using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;
using namespace praxis::scheduler;

namespace {

time_point reading()
{
    return time_point{};
}

clock_source dictating()
{
    return clock_source{&reading};
}

// One publication makes one forward solve for the flange and one for the tool, so this tally of
// forward solves is how many publications happened: a mutation that published twice would double it.
std::uint32_t solves = 0;

praxis::expected<praxis::transform, praxis::refusal> counting_forward_kinematics(const praxis::transform &m, std::span<const praxis::screw_axis> screws, const joint_vector &theta)
{
    ++solves;

    return sliding_forward_kinematics(m, screws, theta);
}

// The answer is the desired pose's own translation, so a command reaches a configuration the case
// can name, and the one recorded step is what the published trace is compared against.
praxis::expected<void, praxis::refusal> recording_inverse_kinematics(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &,
                                                                     const praxis::transform &desired, const joint_vector &j0, const solver_parameters &, ik_result &answer)
{
    answer.iterations.push_back(iteration_state{j0, 0.5, 0.25, 0.125, 7u});
    answer.solutions.push_back(configuration(desired(0, 3), desired(1, 3)));

    return {};
}

praxis::expected<joint_vector, praxis::refusal> solving_task_space_pose(const kinematics &solver, const praxis::transform &pose, const joint_vector &j0)
{
    return solver.ik_solve(pose, j0, solver_parameters{});
}

// Six rows and one column per joint, every entry a value single precision carries exactly, so the
// published matrix is compared against the answered one without a tolerance to hide inside.
praxis::expected<jacobian, praxis::refusal> answering_space_jacobian(std::span<const praxis::screw_axis>, const joint_vector &theta)
{
    jacobian columns(6, theta.size());
    for(Eigen::Index joint = 0; joint < theta.size(); ++joint)
        for(Eigen::Index row = 0; row < 6; ++row)
            columns(row, joint) = 0.25 * static_cast<double>(row) + theta[joint];

    return columns;
}

// The recording double answers the desired pose's own translation, so a case names the configuration
// a command reaches by naming the pose it commands.
praxis::transform reachable(double x, double y)
{
    praxis::transform pose = praxis::transform::Identity();
    pose(0, 3)             = x;
    pose(1, 3)             = y;

    return pose;
}

forward_kinematics_ops counting()
{
    return forward_kinematics_ops{.forward_kinematics = &counting_forward_kinematics};
}

differential_kinematics_ops answering_jacobian()
{
    return differential_kinematics_ops{.space_jacobian = &answering_space_jacobian};
}

inverse_kinematics_ops solving()
{
    return inverse_kinematics_ops{&recording_inverse_kinematics};
}

// What a case reaches the pipe through: the robot, the controller and the publisher are reachable
// from nowhere but the aggregate the gate owns.
struct arm_pipe
{
    strand work;
    std::shared_ptr<owned_arm> owned;
    arm_reader seen;
};

// The seeded configuration is the one every case starts from, over a composition binding neither
// Jacobian slot unless a case hands one in.
arm_pipe pipe(scheduler &loop, const forward_kinematics_ops &forward = counting(), const differential_kinematics_ops &differential = differential_kinematics_ops{},
              const inverse_kinematics_ops &inverse = solving())
{
    const strand work = *loop.make_strand();
    auto robot        = std::make_shared<scene_robot>(
            scene_robot::compose(
                    kinematics::compose(sliding_chain(), forward, differential, inverse, praxis::rigid_motion::baseline().screw, praxis::rigid_motion::baseline().frame).value(),
                    robot_ops{}, praxis::rigid_motion::baseline().frame, 2u)
                    .value());
    robot->set_joint_positions(configuration(0.25, -0.5));

    auto controller = std::make_shared<robot_controller>(*robot, motion_ops{.task_space_pose = &solving_task_space_pose}, composing_path(), task_trajectory_ops{},
                                                         composing_time_scaling(), praxis::trajectory::trajectory_ops{}, praxis::rigid_motion::screw_ops{});
    auto published  = std::make_shared<arm_publisher>();
    auto owned      = std::make_shared<owned_arm>(work, work, robot, controller, published);

    return arm_pipe{work, owned, published->reader()};
}

// Every value commanded in this file is one single precision carries exactly, so the renderer's
// round trip leaves the comparison exact and a substituted value cannot hide inside a tolerance.
void require_joints(const arm_snapshot &seen, double first, double second)
{
    REQUIRE(seen.joints.size() == 2);
    REQUIRE(seen.joints[0] == first);
    REQUIRE(seen.joints[1] == second);
}

}

TEST_CASE("a reader taken at composition already reads a published snapshot rather than an empty one", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm                             = pipe(loop);
    const std::shared_ptr<const arm_snapshot> seen = arm.seen.read();

    REQUIRE(seen != nullptr);
    require_joints(*seen, 0.25, -0.5);
    REQUIRE(seen->tool_position.has_value());
}

TEST_CASE("a command crosses the gate as posted work and does not run on the calling thread", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm           = pipe(loop);
    const joint_vector commanded = configuration(0.75, 1.25);

    command(std::weak_ptr<owned_arm>(arm.owned), [commanded](robot_controller &control, scene_robot &) { control.preview_joint_configuration(commanded); });
    require_joints(*arm.seen.read(), 0.25, -0.5);

    REQUIRE(loop.drain().has_value());
    require_joints(*arm.seen.read(), 0.75, 1.25);
}

TEST_CASE("every mutation publishes exactly once", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm                               = pipe(loop);
    const std::weak_ptr<owned_arm> observer          = arm.owned;
    const std::shared_ptr<const arm_snapshot> seeded = arm.seen.read();

    solves = 0;
    command(observer, [](robot_controller &control, scene_robot &) { control.set_velocity_factor(0.5); });
    REQUIRE(loop.drain().has_value());
    const std::shared_ptr<const arm_snapshot> once = arm.seen.read();

    command(observer, [](robot_controller &control, scene_robot &) { control.set_velocity_factor(0.25); });
    REQUIRE(loop.drain().has_value());
    const std::shared_ptr<const arm_snapshot> twice = arm.seen.read();

    REQUIRE(solves == 4);
    REQUIRE(seeded != once);
    REQUIRE(once != twice);
    REQUIRE(once->velocity_factor == 0.5);
    REQUIRE(twice->velocity_factor == 0.25);
}

TEST_CASE("a command whose observer has expired is not sent and is not reported", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    arm_pipe arm                                   = pipe(loop);
    const std::weak_ptr<owned_arm> observer        = arm.owned;
    const std::shared_ptr<const arm_snapshot> last = arm.seen.read();

    arm.owned.reset();
    REQUIRE(observer.expired());

    solves = 0;
    REQUIRE_NOTHROW(command(observer, [](robot_controller &control, scene_robot &) { control.set_velocity_factor(0.5); }));
    REQUIRE(loop.drain().has_value());

    REQUIRE(solves == 0);
    REQUIRE(arm.seen.read() == last);
}

TEST_CASE("a command posted to a strand whose retirement has been asked for is not sent and is not reported", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm                             = pipe(loop);
    const std::shared_ptr<const arm_snapshot> last = arm.seen.read();

    REQUIRE(loop.retire_strand(arm.work, nullptr).has_value());

    solves = 0;
    command(std::weak_ptr<owned_arm>(arm.owned), [](robot_controller &control, scene_robot &) { control.set_velocity_factor(0.5); });
    REQUIRE(loop.drain().has_value());

    REQUIRE(solves == 0);
    REQUIRE(arm.seen.read() == last);
    REQUIRE(last->velocity_factor == 0.3);
}

TEST_CASE("a command that solved once publishes exactly one iterate sequence", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm = pipe(loop);

    REQUIRE(arm.seen.read()->iterations.empty());

    command(std::weak_ptr<owned_arm>(arm.owned), [](robot_controller &control, scene_robot &) { control.preview_task_space_pose(reachable(0.5, -0.25)); });
    REQUIRE(loop.drain().has_value());

    const std::shared_ptr<const arm_snapshot> seen = arm.seen.read();

    REQUIRE(seen->iterations.size() == 1);
    REQUIRE(seen->iterations.front().size() == 1);
    CHECK(seen->iterations.front().front().index == 7u);
    require_joints(*seen, 0.5, -0.25);
}

// Each sequence carries the configuration its own solve started from, so the two are told apart by
// what they hold rather than by their order alone.
TEST_CASE("a command that solved twice publishes two sequences, each recoverable on its own", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm                      = pipe(loop);
    const std::weak_ptr<owned_arm> observer = arm.owned;

    command(observer,
            [](robot_controller &control, scene_robot &)
            {
                control.preview_task_space_pose(reachable(0.5, -0.25));
                control.preview_task_space_pose(reachable(0.75, 0.5));
            });
    REQUIRE(loop.drain().has_value());

    const std::shared_ptr<const arm_snapshot> twice = arm.seen.read();

    REQUIRE(twice->iterations.size() == 2);
    REQUIRE(twice->iterations[0].size() == 1);
    REQUIRE(twice->iterations[1].size() == 1);
    CHECK(twice->iterations[0].front().joint_positions[0] == 0.25);
    CHECK(twice->iterations[1].front().joint_positions[0] == 0.5);
    require_joints(*twice, 0.75, 0.5);
}

TEST_CASE("a command that hands the solver nothing leaves standing what the command before it left", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm                      = pipe(loop);
    const std::weak_ptr<owned_arm> observer = arm.owned;

    command(observer, [](robot_controller &control, scene_robot &) { control.preview_task_space_pose(reachable(0.5, -0.25)); });
    REQUIRE(loop.drain().has_value());
    REQUIRE(arm.seen.read()->iterations.size() == 1);

    command(observer, [](robot_controller &control, scene_robot &) { control.set_velocity_factor(0.5); });
    REQUIRE(loop.drain().has_value());

    // What stands here belongs to the last command that asked the solver for something, and this one
    // asked for nothing, so it is the sequence before it that a reader still sees.
    const std::shared_ptr<const arm_snapshot> nudged = arm.seen.read();

    REQUIRE(nudged->iterations.size() == 1);
    REQUIRE(nudged->iterations.front().size() == 1);
    CHECK(nudged->iterations.front().front().index == 7u);
}

TEST_CASE("one publication carries the space Jacobian a bound slot answers and a refusal for the body one", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm = pipe(loop, counting(), answering_jacobian());

    command(std::weak_ptr<owned_arm>(arm.owned), [](robot_controller &control, scene_robot &) { control.preview_joint_configuration(configuration(0.75, 1.25)); });
    REQUIRE(loop.drain().has_value());

    const std::shared_ptr<const arm_snapshot> seen = arm.seen.read();

    REQUIRE(seen->space_jacobian.has_value());
    REQUIRE_FALSE(seen->body_jacobian.has_value());
    CHECK(seen->body_jacobian.error() == praxis::refusal::not_implemented);

    const jacobian answered = answering_space_jacobian({}, seen->joints).value();

    REQUIRE(seen->space_jacobian->rows() == 6);
    REQUIRE(seen->space_jacobian->cols() == seen->joints.size());
    CHECK(seen->space_jacobian->cwiseEqual(answered).all());
}

TEST_CASE("a composition binding neither Jacobian slot publishes two refusals and reports nothing", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    std::shared_ptr<const arm_snapshot> seen;

    const std::string reported = reported_by(
            [&loop, &seen]
            {
                const arm_pipe arm = pipe(loop);
                command(std::weak_ptr<owned_arm>(arm.owned), [](robot_controller &control, scene_robot &) { control.set_velocity_factor(0.5); });
                REQUIRE(loop.drain().has_value());
                seen = arm.seen.read();
            });

    REQUIRE(seen != nullptr);
    CHECK_FALSE(seen->space_jacobian.has_value());
    CHECK_FALSE(seen->body_jacobian.has_value());
    CHECK(seen->space_jacobian.error() == praxis::refusal::not_implemented);
    CHECK(seen->body_jacobian.error() == praxis::refusal::not_implemented);
    CHECK(reported.empty());
}
