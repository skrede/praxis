#ifndef HPP_GUARD_PRAXIS_PRESETS_KEPT_MODELING_H
#define HPP_GUARD_PRAXIS_PRESETS_KEPT_MODELING_H

#include "praxis/presets/arm.h"

#include "praxis/manipulator/compose_arm.h"

#include "praxis/config/binding.h"
#include "praxis/config/document.h"

namespace praxis::presets {

// `opening` is the scenario the windows open at and the models are drawn for, so it is the one
// handed to `arm_preset`. A save of the windows' offer writes through `bound`, as the Chain window's
// own save does, and that offer is judged against `carried`, the values the document opened at.
struct kept_modeling
{
    arm_scenario opening;
    config::binding bound;
    config::document carried;
    manipulator::arm_composition composed;
};

// `arm_windows_modeling` over the chain `keeping` keeps, a binding over `screw_table_keyspace()` as
// `screw_table_binding` answers. The View and Tool windows open at what that document carries under
// `window_paths::robot_view` and `window_paths::tool`, each value it does not carry at `chosen`'s, so
// opening leaves nothing unsaved; a document that cannot be read opens at those values throughout.
kept_modeling arm_windows_kept_modeling(const arm_scenario &chosen, const config::binding &keeping, modeling_beside beside = modeling_beside::pose);

}

#endif
