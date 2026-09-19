#ifndef HPP_GUARD_PRAXIS_PRESETS_ARM_ARM_WINDOWS_H
#define HPP_GUARD_PRAXIS_PRESETS_ARM_ARM_WINDOWS_H

#include "praxis/presets/arm.h"

#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <cstddef>

namespace praxis::presets {

// The keys the windows an arm scenario opens carry, declared beneath the paths `window_paths` names
// and read back out from under those same ones, so a window's declaration and its reading stand
// beside each other. A start of a width other than `joints` is declined by name, a document carrying
// no joint count of its own.
void declare_arm_windows(config::declaration &shape);
void read_arm_windows(arm_scenario &read, const config::document &values, std::size_t joints);

}

#endif
