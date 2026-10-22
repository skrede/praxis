#include "fixtures.h"
#include "captured_log.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/optimization/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <cstdint>
#include <utility>

using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;
using namespace praxis::scheduler;

using Catch::Matchers::ContainsSubstring;

namespace {

time_point reading()
{
    return time_point{};
}

robot_ops tool_pose_unbound()
{
    robot_ops ops                  = praxis::manipulator::baseline().robot;
    ops.tool_pose_from_flange_pose = &inert::tool_pose_from_flange_pose;

    return ops;
}

scene_robot flangeless_arm()
{
    return scene_robot::compose(kinematics::compose(sliding_chain(), forward_kinematics_ops{}, differential_kinematics_ops{}, inverse_kinematics_ops{},
                                                    praxis::rigid_motion::baseline().screw, praxis::rigid_motion::baseline().frame, praxis::optimization::baseline().solver)
                                        .value(),
                                tool_pose_unbound(), praxis::rigid_motion::baseline().frame, static_cast<std::uint32_t>(two_joint_handle()->numDOF()))
            .value();
}

robot_controller controller_over(scene_robot &robot)
{
    return robot_controller(robot, composing_motion(), composing_path(), task_trajectory_ops{}, composing_time_scaling(), praxis::trajectory::trajectory_ops{},
                            praxis::rigid_motion::screw_ops{}, praxis::rigid_motion::frame_ops{});
}

struct arm_pipe
{
    strand work;
    std::shared_ptr<owned_arm> owned;
};

arm_pipe pipe(scheduler &loop, scene_robot composed)
{
    const strand work = *loop.make_strand();
    auto robot        = std::make_shared<scene_robot>(std::move(composed));
    auto controller   = std::make_shared<robot_controller>(*robot, motion_ops{}, composing_path(), task_trajectory_ops{}, composing_time_scaling(), praxis::trajectory::trajectory_ops{},
                                                           praxis::rigid_motion::screw_ops{}, praxis::rigid_motion::frame_ops{});
    auto published    = std::make_shared<arm_publisher>();
    auto owned        = std::make_shared<owned_arm>(work, work, robot, controller, published);

    return arm_pipe{work, owned};
}

std::string published_twice(scene_robot composed)
{
    scheduler loop(inline_workers, clock_source{&reading});

    return reported_by(
            [&loop, &composed]
            {
                const arm_pipe arm = pipe(loop, std::move(composed));
                command(std::weak_ptr<owned_arm>(arm.owned), [](robot_controller &control, scene_robot &) { control.set_velocity_factor(0.5); });
                REQUIRE(loop.drain().has_value());
            });
}

}

TEST_CASE("a publication whose flange pose answered and whose tool pose slot refused names that slot once", "[manipulator][refusal]")
{
    const std::string reported = published_twice(two_joint_arm(tool_pose_unbound()));

    CHECK_THAT(reported, ContainsSubstring("'robot.tool_pose_from_flange_pose' is not implemented"));
    CHECK(reported.find("robot.tool_pose_from_flange_pose") == reported.rfind("robot.tool_pose_from_flange_pose"));
    CHECK_FALSE(reported.find("fk.forward_kinematics") != std::string::npos);
}

TEST_CASE("a publication whose flange pose refused names the forward kinematics even though the tool pose slot is unbound too", "[manipulator][refusal]")
{
    const std::string reported = published_twice(flangeless_arm());

    CHECK_THAT(reported, ContainsSubstring("'fk.forward_kinematics' is not implemented"));
    CHECK_FALSE(reported.find("robot.tool_pose_from_flange_pose") != std::string::npos);
}

TEST_CASE("a screw command whose flange pose answered and whose tool pose slot refused names that slot and moves nothing", "[manipulator][refusal]")
{
    scene_robot robot           = two_joint_arm(tool_pose_unbound());
    robot_controller controller = controller_over(robot);
    robot.set_joint_positions(configuration(0.15, 0.25));
    const joint_vector before = robot.joint_positions();

    const std::string reported = reported_by([&] { controller.task_space_screw(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), 0.3, 0.0); });

    CHECK_THAT(reported, ContainsSubstring("'robot.tool_pose_from_flange_pose' is not implemented"));
    CHECK_FALSE(reported.find("fk.forward_kinematics") != std::string::npos);
    CHECK_FALSE(reported.find("screw_axis_from_point_direction_pitch") != std::string::npos);
    CHECK_FALSE(controller.executing());
    CHECK((robot.joint_positions().array() == before.array()).all());
}

TEST_CASE("a screw command whose flange pose refused names the forward kinematics even though the tool pose slot is unbound too", "[manipulator][refusal]")
{
    scene_robot robot           = flangeless_arm();
    robot_controller controller = controller_over(robot);

    const std::string reported = reported_by([&] { controller.task_space_screw(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), 0.3, 0.0); });

    CHECK_THAT(reported, ContainsSubstring("'fk.forward_kinematics' is not implemented"));
    CHECK_FALSE(reported.find("robot.tool_pose_from_flange_pose") != std::string::npos);
    CHECK_FALSE(controller.executing());
}
