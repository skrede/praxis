#ifndef HPP_GUARD_PRAXIS_PRESETS_ARM_ARM_COMPOSERS_H
#define HPP_GUARD_PRAXIS_PRESETS_ARM_ARM_COMPOSERS_H

#include "praxis/presets/arm.h"
#include "praxis/presets/arm_scenarios.h"

#include "praxis/manipulator/compose_arm.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/config/binding.h"
#include "praxis/config/document.h"

#include <optional>

namespace praxis::presets {

// What one composition reads its scenario out of: the arm's own document and the binding it came
// from, and where that arm keeps a chain somebody supplied, if it keeps one anywhere.
struct scenario_documents
{
    config::binding bound;
    config::document carried;
    std::optional<config::binding> keeping;
};

// What one scenario composes -- the windows it opens and the models it draws -- and the document
// those windows write back to, which is the arm's own unless the scenario keeps something of its own
// beside it.
struct opened_scenario
{
    manipulator::arm_composition composed;
    config::binding announced;
    config::document carried;
};

// Which windows a scenario opens, answered from the arm the composition just read and the spatial
// capability it was composed over, so one factory serves every scenario built on an arm. The
// capability is here because how a scenario spells what it keeps is the scenario's own choice of
// binding.
using composer_factory = opened_scenario (*)(const arm_scenario &, const scenario_documents &, const rigid_motion::capabilities &);

composer_factory composer_for(arm_scenario_kind scenario);

}

#endif
