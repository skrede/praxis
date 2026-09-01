#include "demo_machine.h"

#include "praxis/manipulator/configuration.h"
#include "praxis/manipulator/kinematics_configuration.h"
#include "praxis/manipulator/render_configuration.h"
#include "praxis/manipulator/waypoints_configuration.h"
#include "praxis/manipulator/trajectory_configuration.h"
#include "praxis/manipulator/velocity_configuration.h"

#include <cstddef>

namespace praxis::demo {

void declare_windows(config::declaration &shape)
{
    manipulator::declare_tool(shape, presets::window_paths::tool);
    manipulator::declare_world_object(shape, presets::window_paths::world_object);
    manipulator::declare_joint_control(shape, presets::window_paths::joint_control);
    manipulator::declare_task_space(shape, presets::window_paths::task_space);
    manipulator::declare_tool_jog(shape, presets::window_paths::tool_jog);
    manipulator::declare_screw_jog(shape, presets::window_paths::screw_jog);
    manipulator::declare_control_parameters(shape, presets::window_paths::parameters);
    manipulator::declare_recording(shape, presets::window_paths::recording);
    manipulator::declare_robot_view(shape, presets::window_paths::robot_view);
    manipulator::declare_ik_seeds(shape, presets::window_paths::ik_seeds);
    manipulator::declare_ik_branch(shape, presets::window_paths::ik_branch);
    manipulator::declare_ik_iterates(shape, presets::window_paths::ik_iterates);
    manipulator::declare_ik_convergence(shape, presets::window_paths::ik_convergence);
    manipulator::declare_joint_curves(shape, presets::window_paths::joint_curves);
    manipulator::declare_pose_waypoints(shape, presets::window_paths::pose_waypoints);
    manipulator::declare_joint_waypoints(shape, presets::window_paths::joint_waypoints);
    manipulator::declare_path_comparison(shape, presets::window_paths::path_comparison);
    manipulator::declare_trajectory_preview(shape, presets::window_paths::trajectory_preview);
    manipulator::declare_velocity_kinematics(shape, presets::window_paths::velocity_kinematics);
    manipulator::declare_render_controls(shape, presets::window_paths::render_controls);
}

void read_windows(presets::arm_scenario &read, const config::document &values, std::size_t joints)
{
    read.tool                = manipulator::read_tool(values, presets::window_paths::tool);
    read.recording           = manipulator::read_recording(values, presets::window_paths::recording);
    read.tool_jog            = manipulator::read_tool_jog(values, presets::window_paths::tool_jog);
    read.screw_jog           = manipulator::read_screw_jog(values, presets::window_paths::screw_jog);
    read.robot_view          = manipulator::read_robot_view(values, presets::window_paths::robot_view);
    read.task_space          = manipulator::read_task_space(values, presets::window_paths::task_space);
    read.world_object        = manipulator::read_world_object(values, presets::window_paths::world_object);
    read.joint_control       = manipulator::read_joint_control(values, presets::window_paths::joint_control);
    read.parameters          = manipulator::read_control_parameters(values, presets::window_paths::parameters);
    read.ik_seeds            = manipulator::read_ik_seeds(values, presets::window_paths::ik_seeds, joints);
    read.ik_branch           = manipulator::read_ik_branch(values, presets::window_paths::ik_branch);
    read.ik_iterates         = manipulator::read_ik_iterates(values, presets::window_paths::ik_iterates);
    read.ik_convergence      = manipulator::read_ik_convergence(values, presets::window_paths::ik_convergence);
    read.joint_curves        = manipulator::read_joint_curves(values, presets::window_paths::joint_curves);
    read.pose_waypoints      = manipulator::read_pose_waypoints(values, presets::window_paths::pose_waypoints);
    read.joint_waypoints     = manipulator::read_joint_waypoints(values, presets::window_paths::joint_waypoints, joints);
    read.path_comparison     = manipulator::read_path_comparison(values, presets::window_paths::path_comparison, joints);
    read.trajectory_preview  = manipulator::read_trajectory_preview(values, presets::window_paths::trajectory_preview);
    read.velocity_kinematics = manipulator::read_velocity_kinematics(values, presets::window_paths::velocity_kinematics);
    read.render_controls     = manipulator::read_render_controls(values, presets::window_paths::render_controls);
}

}
