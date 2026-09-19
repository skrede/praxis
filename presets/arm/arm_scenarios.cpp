#include "arm_keys.h"
#include "arm_composers.h"

#include "praxis/presets/arm.h"
#include "praxis/presets/screw_table.h"
#include "praxis/presets/arm_scenarios.h"
#include "praxis/presets/arm_registration.h"

#include "praxis/manipulator/compose_arm.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"

#include <span>
#include <array>
#include <cstddef>
#include <optional>

namespace praxis::presets {

namespace {

// In the enumeration's own order, which is what reading a spelling back as a position and casting
// relies on.
constexpr std::array scenario_spellings{
        "every window",          "forward kinematics", "supplied chain",  "tool and world object", "numerical inverse kinematics", "analytic inverse kinematics",
        "point to point motion", "via point motion",   "path comparison", "velocity kinematics"};

constexpr std::size_t scenario_count = static_cast<std::size_t>(arm_scenario_kind::velocity_kinematics) + 1u;

static_assert(scenario_spellings.size() == scenario_count);

// A scenario that keeps nothing of its own writes back to the arm's own document, so it is the same
// answer over a different composer and the table below names the composer rather than a function
// written out per scenario.
template<manipulator::arm_composition (*composer)(arm_scenario)>
opened_scenario windows_over(const arm_scenario &machine, const scenario_documents &documents, const rigid_motion::capabilities &)
{
    return opened_scenario{composer(machine), documents.bound, documents.carried};
}

// A chain typed into this scenario is what its windows write back, so the document that chain is
// kept in is the one announced and every other window beside it is composed with no key path at all.
// An arm keeping no chain announces its own document and keeps nothing.
opened_scenario modeling_windows(const arm_scenario &machine, const scenario_documents &documents, const rigid_motion::capabilities &motions)
{
    if(!documents.keeping)
        return opened_scenario{arm_windows_modeling(machine, screw_table_source{}), documents.bound, documents.carried};

    const config::outcome kept = config::load_or_defaults(*documents.keeping);
    const screw_table_source supplied{screw_table_path, kept.values, screw_table_route(documents.keeping, motions.frame)};

    return opened_scenario{arm_windows_modeling(machine, supplied), *documents.keeping, kept.values};
}

constexpr std::array offered_scenarios{&windows_over<&arm_windows>,
                                       &windows_over<&arm_windows_forward>,
                                       &modeling_windows,
                                       &windows_over<&arm_windows_tooling>,
                                       &windows_over<&arm_windows_numerical_ik>,
                                       &windows_over<&arm_windows_analytic_ik>,
                                       &windows_over<&arm_windows_point_to_point>,
                                       &windows_over<&arm_windows_via_point>,
                                       &windows_over<&arm_windows_path_comparison>,
                                       &windows_over<&arm_windows_velocity_kinematics>};

static_assert(offered_scenarios.size() == scenario_count);

}

std::span<const char *const> arm_scenario_labels()
{
    return scenario_spellings;
}

std::optional<arm_scenario_kind> arm_scenario_named(const config::document &values)
{
    const std::optional<std::size_t> spelling = keys::spelled_index(values, keys::preset_scenario_key, scenario_spellings);
    if(!spelling)
        return std::nullopt;

    return static_cast<arm_scenario_kind>(*spelling);
}

composer_factory composer_for(arm_scenario_kind scenario)
{
    return offered_scenarios.at(static_cast<std::size_t>(scenario));
}

}
