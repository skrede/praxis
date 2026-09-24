#include "arm_ik_windows.h"
#include "arm_pose_transformations.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/robot_controller.h"

#include "praxis/scene/imgui_window.h"

#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <string_view>

namespace praxis::presets {

namespace {

constexpr std::string_view composer_name = "presets.arm_windows_analytic_ik";

using composed_windows = std::vector<std::shared_ptr<scene::imgui_window>>;

composed_windows declined(const std::string &unbound)
{
    spdlog::error("praxis: '{}' was denied {}, which still hold their defaults; every pose it would show or command would be answered without being computed, so it composes no "
                  "window",
                  composer_name, unbound);

    return composed_windows{};
}

// A closed form takes no start: one ask asks for every configuration that reaches the target, and a
// geometry the decomposition has no answer for is refused where the ask was made.
manipulator::ik_branch_window::solve_route in_closed_form()
{
    return [](const transform &target) { return [target](manipulator::robot_controller &control) { control.solve_in_closed_form(target); }; };
}

}

manipulator::arm_composition arm_windows_analytic_ik(arm_scenario chosen)
{
    manipulator::arm_composition composed;
    composed.windows = [state = std::move(chosen)](const manipulator::arm_window_inputs &built)
    {
        if(const std::string unbound = unbound_pose_transformations(built.inert); !unbound.empty())
            return declined(unbound);

        ik::opened_target around = ik::open_target(built, state, composer_name);

        around.opened.push_back(ik::open_branches(built, state, around.pose, in_closed_form()));
        around.opened.push_back(ik::open_view(built, state));

        return std::move(around.opened);
    };

    return composed;
}

}
