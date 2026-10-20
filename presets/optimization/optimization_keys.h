#ifndef HPP_GUARD_PRAXIS_PRESETS_OPTIMIZATION_OPTIMIZATION_KEYS_H
#define HPP_GUARD_PRAXIS_PRESETS_OPTIMIZATION_OPTIMIZATION_KEYS_H

#include "praxis/optimization/scenario_state.h"

#include "praxis/config/document.h"

namespace praxis::presets::keys {

inline constexpr const char *preset_name     = "preset/name";
inline constexpr const char *preset_scenario = "preset/scenario";

// The selected function, its range, the seed and the limit as the document carries them, each one
// the document leaves out at the keyspace's fallback.
optimization::scenario_settings curve_settings(const config::document &values, double tolerance);

}

#endif
