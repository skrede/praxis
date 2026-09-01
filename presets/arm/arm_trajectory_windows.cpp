#include "arm_trajectory_windows.h"

#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/joint_control_window.h"

#include <memory>
#include <utility>
#include <functional>

namespace praxis::presets::motion {

opened_waypoints open_waypoints(const manipulator::arm_window_inputs &built, const arm_scenario &state, std::function<void()> edited)
{
    auto rows = std::make_shared<manipulator::joint_waypoint_list>("Waypoints", built.seen, built.frames, state.joint_waypoints, window_paths::joint_waypoints, std::move(edited));

    return opened_waypoints{rows, {std::make_shared<manipulator::joint_control_window>("Joint control", built.seen, built.arm, state.joint_control, window_paths::joint_control), rows}};
}

std::shared_ptr<scene::imgui_window> open_preview(const manipulator::arm_window_inputs &built, const arm_scenario &state, std::function<void()> asked)
{
    return std::make_shared<manipulator::trajectory_preview_window>("Preview", built.seen, built.arm, built.stencil, std::move(asked), state.trajectory_preview,
                                                                    window_paths::trajectory_preview);
}

std::shared_ptr<scene::imgui_window> open_joint_curves(const manipulator::arm_window_inputs &built, const arm_scenario &state)
{
    return std::make_shared<manipulator::joint_curve_window>("Joint curves", built.seen, state.joint_curves, window_paths::joint_curves);
}

std::shared_ptr<scene::imgui_window> open_view(const manipulator::arm_window_inputs &built, const arm_scenario &state)
{
    manipulator::robot_view_window::controls offered;
    offered.decoration = false;

    return std::make_shared<manipulator::robot_view_window>("View", built.stencil, offered, state.robot_view, window_paths::robot_view);
}

std::function<void()> releasing(std::weak_ptr<manipulator::owned_arm> arm)
{
    return [arm = std::move(arm)] { manipulator::command(arm, [](manipulator::robot_controller &control, manipulator::scene_robot &) { control.clear_preview(); }); };
}

}
