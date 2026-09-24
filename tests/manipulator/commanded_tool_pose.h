#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_COMMANDED_TOOL_POSE_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_COMMANDED_TOOL_POSE_H

#include "three_link_arm.h"

#include "praxis/manipulator/scene_robot.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/baseline/frame.h"

#include <Eigen/Core>

#include <cstdint>
#include <functional>

namespace praxis::fixture {

// The translation lies in the arm's plane, 0.155396 m long, and the turn is about that plane's normal.
inline transform bent_tool()
{
    return rigid_motion::transformation_matrix_from_rotation_position(rigid_motion::rotate_z(0.35), Eigen::Vector3d(0.132, 0.082, 0.0));
}

inline transform straight_tool()
{
    return rigid_motion::transformation_matrix_from_position(Eigen::Vector3d(0.214, 0.0, 0.0));
}

inline transform no_tool()
{
    return transform::Identity();
}

// Metres.
inline double jog_tick()
{
    return 0.005;
}

// Both the flange pose and the tool pose stand well short of the 1.2 m reach at this configuration.
inline joint_vector tooled_configuration()
{
    return arm_configuration(0.4, -1.8, 1.0);
}

// The controller holds a reference to the holder, so the holder is declared first.
class commanded_arm
{
public:
    explicit commanded_arm(const transform &tool_offset, const robot_ops &injected = manipulator::baseline().robot)
            : m_driven(composed(injected))
            , m_control(m_driven, manipulator::baseline().motion, trajectory::baseline().path, manipulator::baseline().trajectory, trajectory::baseline().time_scaling,
                        trajectory::baseline().trajectory, rigid_motion::baseline().screw, rigid_motion::baseline().frame)
    {
        m_driven.set_tool_offset(tool_offset);
        m_driven.set_joint_positions(tooled_configuration());
    }

    scene_robot &driven()
    {
        return m_driven;
    }

    robot_controller &control()
    {
        return m_control;
    }

    transform tool_pose() const
    {
        return m_driven.tool_pose().value();
    }

    Eigen::Vector3d tool_position() const
    {
        return m_driven.tool_position().value();
    }

private:
    scene_robot m_driven;
    robot_controller m_control;

    static scene_robot composed(const robot_ops &injected)
    {
        const kinematics solver = kinematics::compose(three_link_arm(), manipulator::baseline().fk, manipulator::baseline().dk, manipulator::baseline().ik,
                                                      rigid_motion::baseline().screw, rigid_motion::baseline().frame)
                                          .value();

        return scene_robot::compose(solver, injected, rigid_motion::baseline().frame, static_cast<std::uint32_t>(3)).value();
    }
};

inline double tool_travel(commanded_arm &placed, const std::function<void()> &commanding)
{
    const Eigen::Vector3d before = placed.tool_position();
    commanding();

    return (placed.tool_position() - before).norm();
}

}

#endif
