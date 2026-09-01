#ifndef HPP_GUARD_PRAXIS_PRESETS_ARM_ARM_IK_WINDOWS_H
#define HPP_GUARD_PRAXIS_PRESETS_ARM_ARM_IK_WINDOWS_H

#include "praxis/presets/arm.h"

#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/edited_pose.h"
#include "praxis/manipulator/ik_branch_window.h"

#include "praxis/scene/imgui_window.h"

#include <memory>
#include <vector>
#include <string_view>

namespace praxis::presets::ik {

// The windows an inverse-kinematics composition opens before the ones its own solve needs, and the
// pose they stand around. The pose is answered beside them because the list a solve is asked for
// through is handed the same one the target-pose window edits, and a composition that opens a list
// of starts puts that list between the two.
struct opened_target
{
    std::shared_ptr<manipulator::edited_pose> pose;
    std::vector<std::shared_ptr<scene::imgui_window>> opened;
};

// The arm shown with the axes its own description derived, the joint values it is driven by, and the
// pose a solve is asked at. `composer` is what a chain the stencil refuses is reported against.
opened_target open_target(const manipulator::arm_window_inputs &built, const arm_scenario &state, std::string_view composer);

// The answers one solve found, and the control it is asked for through. Which solve one ask makes is
// carried in `asked`, so neither composition names a slot.
std::shared_ptr<scene::imgui_window> open_branches(const manipulator::arm_window_inputs &built, const arm_scenario &state, std::shared_ptr<manipulator::edited_pose> pose,
                                                   manipulator::ik_branch_window::solve_route asked);

std::shared_ptr<scene::imgui_window> open_view(const manipulator::arm_window_inputs &built, const arm_scenario &state);

}

#endif
