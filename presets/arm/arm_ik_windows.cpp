#include "arm_ik_windows.h"

#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <utility>

namespace praxis::presets::ik {

namespace {

// The chain is the one this composition's own description derived, so a refusal here says it names a
// joint count the rendered arm does not have; the stencil names both counts where it compares them.
// The arm is still worth showing, so the windows are composed either way.
void draw_derived_chain(manipulator::loadable_robot_stencil &on, const manipulator::screw_chain &derived, std::string_view composer)
{
    const expected<void, refusal> told = on.set_joint_screws(derived.home, derived.space_screws);
    if(!told)
        spdlog::error("praxis: '{}' was refused the chain its description derived; the arm is shown without its screw axes", composer);
}

manipulator::robot_view_window::controls every_view_control()
{
    manipulator::robot_view_window::controls offered;
    offered.reach = true;

    return offered;
}

}

opened_target open_target(const manipulator::arm_window_inputs &built, const arm_scenario &state, std::string_view composer)
{
    draw_derived_chain(built.stencil, built.chain, composer);
    built.stencil.set_flange_attachment(manipulator::flange_attachment::frame_marker, manipulator::make_flange_marker(built.stencil.robot()));

    auto pose = std::make_shared<manipulator::edited_pose>();

    return opened_target{pose,
                         {std::make_shared<manipulator::joint_control_window>("Joint control", built.seen, built.arm, state.joint_control, window_paths::joint_control),
                          std::make_shared<manipulator::task_space_window>("Target pose", built.seen, built.arm, built.frames, pose, state.task_space, window_paths::task_space)}};
}

std::shared_ptr<scene::imgui_window> open_branches(const manipulator::arm_window_inputs &built, const arm_scenario &state, std::shared_ptr<manipulator::edited_pose> pose,
                                                   manipulator::ik_branch_window::solve_route asked)
{
    return std::make_shared<manipulator::ik_branch_window>("Solutions", built.seen, built.arm, built.frames, std::move(pose), built.stencil, std::move(asked), state.ik_branch,
                                                           window_paths::ik_branch);
}

std::shared_ptr<scene::imgui_window> open_view(const manipulator::arm_window_inputs &built, const arm_scenario &state)
{
    return std::make_shared<manipulator::robot_view_window>("View", built.stencil, every_view_control(), state.robot_view, window_paths::robot_view);
}

}
