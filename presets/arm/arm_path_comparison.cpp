#include "arm_trajectory_windows.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/path_comparison_window.h"

#include "praxis/scene/imgui_window.h"

#include <memory>
#include <vector>
#include <utility>

namespace praxis::presets {

manipulator::arm_composition arm_windows_path_comparison(arm_scenario chosen)
{
    manipulator::arm_composition composed;
    composed.windows = [state = std::move(chosen)](const manipulator::arm_window_inputs &built)
    {
        std::vector<std::shared_ptr<scene::imgui_window>> opened;
        opened.push_back(std::make_shared<manipulator::joint_control_window>("Joint control", built.seen, built.arm, state.joint_control, window_paths::joint_control));
        opened.push_back(std::make_shared<manipulator::path_comparison_window>("Comparison", built.seen, built.arm, built.stencil, built.fk, built.chain, built.path,
                                                                               state.path_comparison, window_paths::path_comparison));
        opened.push_back(motion::open_view(built, state));

        return opened;
    };

    return composed;
}

}
