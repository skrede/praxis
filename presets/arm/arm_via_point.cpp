#include "arm_trajectory_windows.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/robot_controller.h"

#include "praxis/scene/imgui_window.h"

#include <span>
#include <memory>
#include <vector>
#include <utility>
#include <functional>

namespace praxis::presets {

namespace {

// The list is read where the ask is made, on the strand it is edited on, and only the rows the
// route answers cross onto the arm's own strand.
std::function<void()> through_the_waypoints(std::weak_ptr<manipulator::owned_arm> arm, std::shared_ptr<manipulator::joint_waypoint_list> listed)
{
    return [arm = std::move(arm), listed = std::move(listed)]
    {
        const std::vector<manipulator::joint_vector> rows = listed->state().rows;
        manipulator::command(arm, [rows](manipulator::robot_controller &control, manipulator::scene_robot &)
                             { control.preview_trajectory(std::span<const manipulator::joint_vector>(rows)); });
    };
}

}

manipulator::arm_composition arm_windows_via_point(arm_scenario chosen)
{
    manipulator::arm_composition composed;
    composed.windows = [state = std::move(chosen)](const manipulator::arm_window_inputs &built)
    {
        motion::opened_waypoints around = motion::open_waypoints(built, state, motion::releasing(built.arm));

        around.opened.push_back(motion::open_preview(built, state, through_the_waypoints(built.arm, around.rows)));
        around.opened.push_back(motion::open_joint_curves(built, state));

        return std::move(around.opened);
    };

    return composed;
}

}
