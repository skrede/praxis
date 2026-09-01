#include "praxis/presets/arm.h"

#include "praxis/manipulator/robot.h"
#include "praxis/manipulator/slots.h"
#include "praxis/manipulator/pose_readout.h"
#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/extension/coverage.h"

#include "praxis/scene/imgui_window.h"

#include <spdlog/spdlog.h>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::presets {

namespace {

using composed_windows = std::vector<std::shared_ptr<scene::imgui_window>>;

// The four frame transformations every pose this scenario shows is read through. Each of them
// answers the origin unrotated where nobody bound it, and the origin unrotated is a pose an arm can
// genuinely be at, so downstream of them a fabrication and a reading are the same value.
constexpr std::array<manipulator::robot_slot, 4> pose_transformations{manipulator::robot_slot::tool_pose_from_flange_pose, manipulator::robot_slot::flange_pose_from_tool_pose,
                                                                      manipulator::robot_slot::position_from_pose, manipulator::robot_slot::orientation_from_pose};

// The names come from the same descriptor table the coverage report reads, so a slot renamed in one
// place is named the same way here.
constexpr manipulator::robot_ops slot_names{};

std::string unbound_among(const manipulator::robot_slot_set &inert)
{
    const capability_view described = view_of(slot_names);

    std::string listed;
    for(const manipulator::robot_slot needed : pose_transformations)
        if(inert.contains(needed))
        {
            if(!listed.empty())
                listed += ", ";
            listed += slot_name(described, static_cast<std::size_t>(needed));
        }

    return listed;
}

// Said once, at composition, rather than branched on at every read: the four transformations cannot
// refuse, so a window opened over an unbound one would show a plausible pose forever and no reader
// downstream could tell. Only the slots this scenario itself reads are counted, so binding a solver
// is not the price of attaching a tool. The models were chosen before this was reached, and a
// composition offering no window to hide them with draws neither.
composed_windows declined(manipulator::loadable_robot_stencil &on, const std::string &unbound)
{
    on.clear_flange_attachment(manipulator::flange_attachment::tool);
    on.clear_world_object();

    spdlog::error("praxis: 'presets.arm_windows_tooling' was denied {}, which still hold their defaults; every pose it would show would be answered without being computed, so it "
                  "composes no window and draws neither model",
                  unbound);

    return composed_windows{};
}

// The chain is the one this composition's own description derived, so a refusal here says it names a
// joint count the rendered arm does not have; the stencil names both counts where it compares them.
// The arm is still worth showing, so the windows are composed either way.
void draw_derived_chain(manipulator::loadable_robot_stencil &on, const manipulator::screw_chain &derived)
{
    const expected<void, refusal> told = on.set_joint_screws(derived.home, derived.space_screws);
    if(!told)
        spdlog::error("praxis: 'presets.arm_windows_tooling' was refused the chain its description derived; the arm is shown without its screw axes");
}

manipulator::robot_view_window::controls every_view_control()
{
    manipulator::robot_view_window::controls offered;
    offered.reach = true;

    return offered;
}

}

manipulator::arm_composition arm_windows_tooling(arm_scenario chosen)
{
    manipulator::arm_composition composed;
    composed.draws_tool  = true;
    composed.draws_world = true;
    composed.windows     = [state = std::move(chosen)](const manipulator::arm_window_inputs &built)
    {
        if(const std::string unbound = unbound_among(built.inert); !unbound.empty())
            return declined(built.stencil, unbound);

        draw_derived_chain(built.stencil, built.chain);
        built.stencil.set_flange_attachment(manipulator::flange_attachment::frame_marker, manipulator::make_flange_marker(built.stencil.robot()));

        return composed_windows{
                std::make_shared<manipulator::joint_control_window>("Joint control", built.seen, built.arm, state.joint_control, window_paths::joint_control),
                manipulator::compose_pose_readout("Pose", built.seen, built.frames, built.inert),
                std::make_shared<manipulator::tool_window>("Tool", built.stencil, built.seen, built.arm, built.frames, state.tool, window_paths::tool),
                std::make_shared<manipulator::world_object_window>("World object", built.stencil, built.frames, state.world_object, window_paths::world_object),
                std::make_shared<manipulator::robot_view_window>("View", built.stencil, every_view_control(), state.robot_view, window_paths::robot_view),
        };
    };

    return composed;
}

}
