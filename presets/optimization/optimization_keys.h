#ifndef HPP_GUARD_PRAXIS_PRESETS_OPTIMIZATION_OPTIMIZATION_KEYS_H
#define HPP_GUARD_PRAXIS_PRESETS_OPTIMIZATION_OPTIMIZATION_KEYS_H

#include "praxis/optimization/scenario_state.h"

#include "praxis/config/document.h"

#include <cstddef>

namespace praxis::presets::keys {

inline constexpr const char *preset_name     = "preset/name";
inline constexpr const char *preset_scenario = "preset/scenario";

// The selected function, a range and a seed coordinate for each of its `dimension` variables, the
// limit, the form, the method, the step length, the source, the typed text and its parameters in
// letter order as the document carries them, each one the document leaves out at the keyspace's
// fallback.
optimization::scenario_settings opened_settings(const config::document &values, std::size_t dimension);

}

#endif
