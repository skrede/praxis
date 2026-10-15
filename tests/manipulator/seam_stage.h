#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_SEAM_STAGE_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_SEAM_STAGE_H

#include "fixtures.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/scheduler/task.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include <memory>
#include <cstdint>
#include <utility>

namespace praxis::fixture {

struct composed_arm
{
    std::shared_ptr<manipulator::scene_robot> robot;
    std::unique_ptr<manipulator::robot_controller> control;
};

// The arm and its controller composed the way a course composes them: every slot the reference
// except the trajectory aggregate handed in, and a pose resolved to its own translation.
inline composed_arm arm_under(const trajectory::capabilities &shapes)
{
    const manipulator::capabilities arm    = manipulator::baseline();
    const rigid_motion::capabilities rigid = rigid_motion::baseline();

    auto robot = std::make_shared<manipulator::scene_robot>(two_joint_arm(arm.robot));
    robot->set_joint_positions(configuration(0.0, 0.0));
    auto control =
            std::make_unique<manipulator::robot_controller>(*robot, composing_motion(), shapes.path, arm.trajectory, shapes.time_scaling, shapes.trajectory, rigid.screw, rigid.frame);

    return composed_arm{std::move(robot), std::move(control)};
}

inline bool played_out(manipulator::robot_controller &control)
{
    constexpr scheduler::step_delta stepped{scheduler::seconds{0.01}};
    constexpr std::uint32_t most_steps = 100000;

    for(std::uint32_t taken = 0; taken < most_steps && control.executing(); ++taken)
        static_cast<void>(control.advance_playback(stepped));

    return !control.executing();
}

}

#endif
