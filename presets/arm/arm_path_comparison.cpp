#include "arm_trajectory_windows.h"
#include "arm_pose_transformations.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/path_comparison_window.h"

#include "praxis/scene/imgui_window.h"

#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <string_view>

namespace praxis::presets {

namespace {

constexpr std::string_view composer_name = "presets.arm_windows_path_comparison";

using composed_windows = std::vector<std::shared_ptr<scene::imgui_window>>;

composed_windows declined(const std::string &unbound)
{
    spdlog::error("praxis: '{}' was denied {}, which still hold their defaults; every pose it would show or command would be answered without being computed, so it composes no "
                  "window",
                  composer_name, unbound);

    return composed_windows{};
}

}

manipulator::arm_composition arm_windows_path_comparison(arm_scenario chosen)
{
    manipulator::arm_composition composed;
    composed.windows = [state = std::move(chosen)](const manipulator::arm_window_inputs &built)
    {
        if(const std::string unbound = unbound_pose_transformations(built.inert); !unbound.empty())
            return declined(unbound);

        composed_windows opened;
        opened.push_back(std::make_shared<manipulator::joint_control_window>("Joint control", built.seen, built.arm, state.joint_control, window_paths::joint_control));
        opened.push_back(std::make_shared<manipulator::path_comparison_window>("Comparison", built.seen, built.arm, built.stencil, built.screw, built.fk, built.chain, built.path,
                                                                               built.robot, state.path_comparison, window_paths::path_comparison));
        opened.push_back(motion::open_view(built, state));

        return opened;
    };

    return composed;
}

}
