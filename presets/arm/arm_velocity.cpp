#include "praxis/presets/arm.h"

#include "praxis/manipulator/slots.h"
#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/joint_control_window.h"
#include "praxis/manipulator/render_controls_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"
#include "praxis/manipulator/velocity_kinematics_window.h"

#include "praxis/scene/imgui_window.h"

#include <spdlog/spdlog.h>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace praxis::presets {

namespace {

using composed_windows = std::vector<std::shared_ptr<scene::imgui_window>>;

// Every quantity this scenario shows is taken from these two, and each refuses where nobody bound
// it.
constexpr std::array<manipulator::differential_kinematics_slot, 2> jacobians{manipulator::differential_kinematics_slot::space_jacobian,
                                                                             manipulator::differential_kinematics_slot::body_jacobian};

constexpr manipulator::differential_kinematics_ops slot_names{};

// Said once at composition, rather than read out of a published snapshot after the arm has
// published.
composed_windows declined(const std::string &unbound)
{
    spdlog::error("praxis: 'arm_windows_velocity_kinematics' was denied {}, which still hold their defaults; every velocity it would show would be a refusal, so it composes no window",
                  unbound);

    return composed_windows{};
}

// The chain and the count of columns drawn from it are both this composition's own description's, so
// a refusal here says the description names a joint count the rendered arm does not have; the
// stencil names both counts where it compares them. The arm is still worth showing, so the windows
// are composed either way.
void raise_structure(manipulator::loadable_robot_stencil &on, const manipulator::screw_chain &derived)
{
    if(!on.set_joint_screws(derived.home, derived.space_screws))
        spdlog::error("praxis: 'arm_windows_velocity_kinematics' was refused the chain its description derived; the arm is shown without its screw axes");
    if(!on.set_jacobian_columns(derived.space_screws.size()))
        spdlog::error("praxis: 'arm_windows_velocity_kinematics' was refused the column count its description derived; the arm is shown without its Jacobian columns");
}

// The screw axes are drawn beside the column arrows, so the reach they are drawn to is reachable.
manipulator::robot_view_window::controls every_view_control()
{
    manipulator::robot_view_window::controls offered;
    offered.reach = true;

    return offered;
}

}

manipulator::arm_composition arm_windows_velocity_kinematics(arm_scenario chosen)
{
    manipulator::arm_composition composed;
    composed.windows = [state = std::move(chosen)](const manipulator::arm_window_inputs &built)
    {
        if(const manipulator::differential_kinematics_slot_set unbound = defaulted_among(built.dk_inert, jacobians); !unbound.empty())
            return declined(joined_slot_names(slot_names, unbound));

        raise_structure(built.stencil, built.chain);

        composed_windows opened;
        opened.push_back(std::make_shared<manipulator::joint_control_window>("Joint control", built.seen, built.arm, state.joint_control, window_paths::joint_control));
        opened.push_back(std::make_shared<manipulator::velocity_kinematics_window>("Velocity kinematics", built.seen, built.arm, built.stencil,
                                                                                   manipulator::velocity_kinematics_window::controls{}, state.velocity_kinematics,
                                                                                   window_paths::velocity_kinematics));
        opened.push_back(std::make_shared<manipulator::render_controls_window>("Render controls", built.stencil, manipulator::render_controls_window::controls{}, state.render_controls,
                                                                               window_paths::render_controls));
        opened.push_back(std::make_shared<manipulator::robot_view_window>("View", built.stencil, every_view_control(), state.robot_view, window_paths::robot_view));

        return opened;
    };

    return composed;
}

}
