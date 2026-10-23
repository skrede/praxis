#ifndef HPP_GUARD_PRAXIS_PRESETS_OPTIMIZATION_WINDOW_PLACEMENTS_H
#define HPP_GUARD_PRAXIS_PRESETS_OPTIMIZATION_WINDOW_PLACEMENTS_H

#include "praxis/scene/imgui_window.h"

#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::presets {

// Where a window of the robot-free scenario opens the first time, by its title and the number of
// variables the scenario's function takes; a title the scenario composes no window under answers
// nothing.
std::optional<scene::opening_placement> scenario_window_placement(std::string_view title, std::size_t dimension);

}

#endif
