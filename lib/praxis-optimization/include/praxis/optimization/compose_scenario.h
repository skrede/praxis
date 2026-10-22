#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_COMPOSE_SCENARIO_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_COMPOSE_SCENARIO_H

#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_view.h"
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
// settings of its own starts from, and `checks_derivative` is the composition's own. `drawn` is what
// the scenario's drawing shows, which a window may switch.
struct scenario_window_inputs
{
    scenario_reader seen;
    std::size_t dimension;
    bool checks_derivative;
    scenario_settings opened;
    std::vector<std::string> registered;
    std::weak_ptr<owned_scenario> scenario;
    std::shared_ptr<scenario_view> drawn;
};

// Invoked once per composition, after the publisher and the gated state exist and before the preset
// is constructed.
using scenario_window_composer = std::function<std::vector<std::shared_ptr<scene::imgui_window>>(const scenario_window_inputs &)>;

// A composition checking derivatives has each linearization's registered derivative compared with a
// central difference of the value; one that does not has nothing of the check evaluated or shown.
struct scenario_composition
{
    scenario_window_composer windows;
    bool checks_derivative = false;
};

// Each answers nothing, and logs why, where the settings' source is registered and they select a
// function the list does not register, carry other than one range and one seed coordinate per
// variable, carry a range that is not finite or whose minimum does not lie below its maximum or a seed
// that is not finite, or the work strand admits no run. A typed text opens with one parameter per
// letter it uses; one that does not read is kept, and nothing is drawn or solved from it.
std::shared_ptr<scene::preset> compose_curve_scenario(const scene::preset_site &site, std::vector<registered_curve> curves, const capabilities &solving, const scenario_settings &opened,
                                                      const scenario_composition &composed);

// The surface is drawn in the site's scene as a height field with every iterate on it.
std::shared_ptr<scene::preset> compose_surface_scenario(const scene::preset_site &site, std::vector<registered_surface> surfaces, const capabilities &solving,
                                                        const scenario_settings &opened, const scenario_composition &composed);

}

#endif
