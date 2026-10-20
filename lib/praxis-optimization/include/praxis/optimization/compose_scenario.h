#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_COMPOSE_SCENARIO_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_COMPOSE_SCENARIO_H

#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <functional>

namespace praxis::optimization {

// What a window of one composed scenario is built from. It carries no function and no solver, so a
// window has nothing it could evaluate or solve on the render strand: it reads the publication and
// asks for a run through the gate. `opened` is what the composition opened at, which a window holding
// settings of its own starts from.
struct scenario_window_inputs
{
    scenario_reader seen;
    std::size_t dimension;
    scenario_settings opened;
    std::vector<std::string> registered;
    std::weak_ptr<owned_scenario> scenario;
};

// Invoked once per composition, after the publisher and the gated state exist and before the preset
// is constructed.
using scenario_window_composer = std::function<std::vector<std::shared_ptr<scene::imgui_window>>(const scenario_window_inputs &)>;

struct scenario_composition
{
    scenario_window_composer windows;
    bool checks_derivative = false;
};

// Answers nothing, and logs why, where the settings select a function the list does not register or
// the work strand admits no run.
std::shared_ptr<scene::preset> compose_curve_scenario(const scene::preset_site &site, std::vector<registered_curve> curves, const capabilities &solving, const scenario_settings &opened,
                                                      const scenario_composition &composed);

}

#endif
