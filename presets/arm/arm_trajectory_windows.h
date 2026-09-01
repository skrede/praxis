#ifndef HPP_GUARD_PRAXIS_PRESETS_ARM_ARM_TRAJECTORY_WINDOWS_H
#define HPP_GUARD_PRAXIS_PRESETS_ARM_ARM_TRAJECTORY_WINDOWS_H

#include "praxis/presets/arm.h"

#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/edited_list_window.h"
#include "praxis/manipulator/trajectory_preview_window.h"

#include "praxis/scene/imgui_window.h"

#include <memory>
#include <vector>
#include <functional>

namespace praxis::presets::motion {

// The waypoints a composition of a generated motion opens, and the run of windows they stand in.
// The list is answered beside them because the ask is made through a route that reads the same rows
// the list holds, so a composer needs the window itself and not only the fact that it was opened.
struct opened_waypoints
{
    std::shared_ptr<manipulator::joint_waypoint_list> rows;
    std::vector<std::shared_ptr<scene::imgui_window>> opened;
};

// The arm's joint values beside the waypoints a motion is composed from. `edited` is told wherever
// a row changes, so a composition routes that to whatever it computed from the rows.
opened_waypoints open_waypoints(const manipulator::arm_window_inputs &built, const arm_scenario &state, std::function<void()> edited);

// The motion asked for before it is played, with its path drawn and its scaling plotted. What one
// ask previews is carried in `asked`, so neither composition names a slot.
std::shared_ptr<scene::imgui_window> open_preview(const manipulator::arm_window_inputs &built, const arm_scenario &state, std::function<void()> asked);

// Each joint of whatever motion stands previewed drawn against time, one curve per joint per frame.
std::shared_ptr<scene::imgui_window> open_joint_curves(const manipulator::arm_window_inputs &built, const arm_scenario &state);

// Which drawing of the arm stands, and nothing else: no control over the drawn screw axes and none
// over how far one of them reaches.
std::shared_ptr<scene::imgui_window> open_view(const manipulator::arm_window_inputs &built, const arm_scenario &state);

// What a composition routes an input a standing preview was computed from to. It is two-layer like
// every other crossing: the outer closure runs where the input is edited and the release is the one
// thing that reaches the arm's own strand.
std::function<void()> releasing(std::weak_ptr<manipulator::owned_arm> arm);

}

#endif
