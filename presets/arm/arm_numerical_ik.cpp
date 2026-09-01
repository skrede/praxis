#include "arm_ik_windows.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/ik_seed_window.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/ik_iterate_window.h"
#include "praxis/manipulator/ik_convergence_window.h"

#include "praxis/scene/imgui_window.h"

#include <memory>
#include <vector>
#include <utility>
#include <string_view>

namespace praxis::presets {

namespace {

constexpr std::string_view composer_name = "presets.arm_windows_numerical_ik";

// The list is read where the ask is made, on the strand it is edited on, and only the solve the
// route answers crosses onto the arm's own strand.
manipulator::ik_branch_window::solve_route from_the_starts(std::shared_ptr<manipulator::ik_seed_window> listed)
{
    return [listed = std::move(listed)](const transform &target)
    { return [target, seeds = listed->state().seeds](manipulator::robot_controller &control) { control.solve_from_seeds(target, seeds); }; };
}

}

manipulator::arm_composition arm_windows_numerical_ik(arm_scenario chosen)
{
    manipulator::arm_composition composed;
    composed.windows = [state = std::move(chosen)](const manipulator::arm_window_inputs &built)
    {
        ik::opened_target around = ik::open_target(built, state, composer_name);

        auto starts = std::make_shared<manipulator::ik_seed_window>("Starts", built.seen, state.ik_seeds, window_paths::ik_seeds);
        auto steps  = std::make_shared<manipulator::ik_iterate_window>("Iterations", built.seen, built.arm, state.ik_iterates, window_paths::ik_iterates);

        around.opened.push_back(starts);
        around.opened.push_back(ik::open_branches(built, state, around.pose, from_the_starts(starts)));
        around.opened.push_back(steps);
        around.opened.push_back(std::make_shared<manipulator::ik_convergence_window>("Convergence", *steps, state.ik_convergence, window_paths::ik_convergence));
        around.opened.push_back(ik::open_view(built, state));

        return std::move(around.opened);
    };

    return composed;
}

}
