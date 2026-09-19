#ifndef HPP_GUARD_PRAXIS_PRESETS_ARM_SCENARIOS_H
#define HPP_GUARD_PRAXIS_PRESETS_ARM_SCENARIOS_H

#include "praxis/config/document.h"

#include <cstdint>
#include <optional>

namespace praxis::presets {

// The order is the label table's, which is what reading a spelling back as a position and casting
// relies on.
enum class arm_scenario_kind : std::uint8_t
{
    every_window,
    forward_kinematics,
    supplied_chain,
    tooling,
    numerical_ik,
    analytic_ik,
    point_to_point,
    via_point,
    path_comparison,
    velocity_kinematics
};

std::optional<arm_scenario_kind> arm_scenario_named(const config::document &values);

}

#endif
