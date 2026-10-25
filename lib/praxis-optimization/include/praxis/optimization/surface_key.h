#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_KEY_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_KEY_H

#include "praxis/optimization/compose_scenario.h"

#include "praxis/scene/key_window.h"

#include <memory>
#include <string>

namespace praxis::optimization {

// A key naming, each in its tone, the figures a composed surface's drawing showed on its last frame.
std::shared_ptr<scene::key_window> surface_key(const scenario_window_inputs &inputs, std::string name);

}

#endif
