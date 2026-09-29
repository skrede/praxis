#include "arm_keys.h"
#include "arm_composers.h"

#include "praxis/presets/arm.h"
#include "praxis/presets/screw_table.h"
#include "praxis/presets/arm_scenarios.h"
#include "praxis/presets/arm_registration.h"

#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/tool_configuration.h"
#include "praxis/manipulator/view_configuration.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <span>
#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>

namespace praxis::presets {

namespace {

// In the enumeration's own order, which is what reading a spelling back as a position and casting
// relies on.
constexpr std::array scenario_spellings{
        "every window",          "forward kinematics", "supplied chain",  "tool and world object", "numerical inverse kinematics", "analytic inverse kinematics",
        "point to point motion", "via point motion",   "path comparison", "velocity kinematics",   "supplied chain and tool"};

constexpr std::size_t scenario_count = static_cast<std::size_t>(arm_scenario_kind::supplied_chain_and_tool) + 1u;

static_assert(scenario_spellings.size() == scenario_count);

// A scenario that keeps nothing of its own writes back to the arm's own document, so it is the same
// answer over a different composer and the table below names the composer rather than a function
// written out per scenario.
template<manipulator::arm_composition (*composer)(arm_scenario)>
opened_scenario windows_over(const arm_scenario &machine, const scenario_documents &documents, const rigid_motion::capabilities &)
{
    return opened_scenario{composer(machine), documents.bound, documents.carried, machine};
}

// A document carrying none of these keys reads what the caller opened at.
config::declaration falling_back_to(const config::declaration &declared, std::span<const config::edit> opened)
{
    config::declaration shape(declared.space());
    for(const config::node &held : declared.nodes())
    {
        const auto named     = std::ranges::find(opened, held.path, &config::edit::key);
        std::string fallback = named == opened.end() ? held.fallback : named->value;
        if(held.shape == config::node_kind::group)
            shape.group(held.path);
        else if(held.shape == config::node_kind::collection)
            shape.collection(held.path, held.identity);
        else if(!held.allowed.empty())
            shape.choice(held.path, held.allowed, std::move(fallback));
        else
            shape.field(held.path, held.kind, std::move(fallback));
    }

    return shape;
}

std::vector<config::edit> opened_beside_the_chain(const arm_scenario &machine)
{
    std::vector<config::edit> opened       = manipulator::write_robot_view(machine.robot_view, window_paths::robot_view);
    const std::vector<config::edit> tooled = manipulator::write_tool(machine.tool, window_paths::tool);
    opened.insert(opened.end(), tooled.begin(), tooled.end());

    return opened;
}

arm_scenario reopened_beside_the_chain(const arm_scenario &machine, const config::document &kept)
{
    arm_scenario opening = machine;
    opening.robot_view   = manipulator::read_robot_view(kept, window_paths::robot_view);
    opening.tool         = manipulator::read_tool(kept, window_paths::tool);

    return opening;
}

// A chain typed into this scenario is what its windows write back, so the document that chain is
// kept in is announced, and the View and Tool windows beside the chain open at what it keeps for
// them, value by value over what the arm's own document names. An arm keeping no chain announces
// its own document.
template<modeling_beside beside>
opened_scenario modeling_windows(const arm_scenario &machine, const scenario_documents &documents, const rigid_motion::capabilities &)
{
    if(!documents.keeping)
        return opened_scenario{arm_windows_modeling(machine, screw_table_source{}, beside), documents.bound, documents.carried, machine};

    const config::binding kept_at{falling_back_to(documents.keeping->shape, opened_beside_the_chain(machine)), documents.keeping->at, documents.keeping->carries};
    const config::outcome kept = config::load_or_defaults(kept_at);
    const arm_scenario opening = reopened_beside_the_chain(machine, kept.values);
    const screw_table_source supplied{screw_table_path, kept.values, kept_at};

    return opened_scenario{arm_windows_modeling(opening, supplied, beside), kept_at, kept.values, opening};
}

constexpr std::array offered_scenarios{&windows_over<&arm_windows>,
                                       &windows_over<&arm_windows_forward>,
                                       &modeling_windows<modeling_beside::pose>,
                                       &windows_over<&arm_windows_tooling>,
                                       &windows_over<&arm_windows_numerical_ik>,
                                       &windows_over<&arm_windows_analytic_ik>,
                                       &windows_over<&arm_windows_point_to_point>,
                                       &windows_over<&arm_windows_via_point>,
                                       &windows_over<&arm_windows_path_comparison>,
                                       &windows_over<&arm_windows_velocity_kinematics>,
                                       &modeling_windows<modeling_beside::pose_and_tool>};

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
