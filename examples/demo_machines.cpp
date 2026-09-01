#include "demo_machines.h"

#include "demo_machine.h"
#include "demo_write_back.h"
#include "demo_configuration.h"

#include "praxis/manipulator/capabilities.h"

#include "praxis/presets/arm.h"

#include "praxis/scene/coverage_report.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/config/binding.h"

#include <spdlog/spdlog.h>

#include <array>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <cstddef>
#include <optional>
#include <filesystem>

namespace praxis::demo {

namespace {

// What this scenario composes and reports on. It is the demonstration's own glue: the library
// publishes the capabilities, and which of them a scenario binds is the scenario's choice.
struct composed_capabilities
{
    manipulator::capabilities arm;
    trajectory::capabilities shapes;
    rigid_motion::capabilities motions;
};

// The report is over what this scenario composed and nothing else, so its width is the number of
// capabilities above rather than anything the library fixes.
std::array<capability_view, 13> composed_views(const composed_capabilities &composed)
{
    return {view_of(composed.arm.fk),
            view_of(composed.arm.dk),
            view_of(composed.arm.ik),
            view_of(composed.arm.robot),
            view_of(composed.arm.motion),
            view_of(composed.arm.modeling),
            view_of(composed.arm.trajectory),
            view_of(composed.shapes.time_scaling),
            view_of(composed.shapes.path),
            view_of(composed.shapes.pose_trajectory),
            view_of(composed.shapes.trajectory),
            view_of(composed.motions.frame),
            view_of(composed.motions.screw)};
}

// The views point into the value passed, so a temporary would leave every one of them dangling at the
// end of the full expression.
std::array<capability_view, 13> composed_views(composed_capabilities &&) = delete;

composed_capabilities bound_capabilities()
{
    return composed_capabilities{manipulator::baseline(), trajectory::baseline(), rigid_motion::baseline()};
}

// What one composition reads its scenario out of: the machine's own document and the binding it
// came from, and where that machine keeps a chain somebody supplied, if it keeps one anywhere.
struct scenario_documents
{
    config::binding bound;
    config::document carried;
    std::optional<config::binding> keeping;
};

// What one scenario composes -- the windows it opens and the models it draws -- and the document
// those windows write back to, which is the machine's own unless the scenario keeps something of its
// own beside it.
struct opened_scenario
{
    manipulator::arm_composition composed;
    config::binding announced;
    config::document carried;
};

// Which windows a scenario opens, answered from the machine the composition just read and the
// spatial capability it was composed over, so one factory below serves every scenario built on an
// arm. The capability is here because how a scenario spells what it keeps is the scenario's own
// choice of binding, not something the library fixes.
using composer_factory = opened_scenario (*)(const presets::arm_scenario &, const scenario_documents &, const rigid_motion::capabilities &);

// A scenario that keeps nothing of its own writes back to the machine's own document, so it is the
// same answer over a different composer and the table below names the composer rather than a
// function written out per scenario.
template<manipulator::arm_composition (*composer)(presets::arm_scenario)>
opened_scenario windows_over(const presets::arm_scenario &machine, const scenario_documents &documents, const rigid_motion::capabilities &)
{
    return opened_scenario{composer(machine), documents.bound, documents.carried};
}

// A chain typed into this scenario is what its windows write back, so the document that chain is
// kept in is the one announced and every other window beside it is composed with no key path at
// all. A machine keeping no chain announces its own document and keeps nothing.
opened_scenario modeling_windows(const presets::arm_scenario &machine, const scenario_documents &documents, const rigid_motion::capabilities &motions)
{
    if(!documents.keeping)
        return opened_scenario{presets::arm_windows_modeling(machine, presets::screw_table_source{}), documents.bound, documents.carried};

    const config::outcome kept = config::load_or_defaults(*documents.keeping);
    const presets::screw_table_source supplied{presets::screw_table_path, kept.values, presets::screw_table_route(documents.keeping, motions.frame)};

    return opened_scenario{presets::arm_windows_modeling(machine, supplied), *documents.keeping, kept.values};
}

// In the order of the spellings a document names a scenario by, which is what reading one back as an
// index and indexing this table relies on. The extent is the entries', so a scenario the spellings
// name and this table does not is a compile error here.
constexpr std::array offered_scenarios{&windows_over<&presets::arm_windows>,
                                       &windows_over<&presets::arm_windows_forward>,
                                       &modeling_windows,
                                       &windows_over<&presets::arm_windows_tooling>,
                                       &windows_over<&presets::arm_windows_numerical_ik>,
                                       &windows_over<&presets::arm_windows_analytic_ik>,
                                       &windows_over<&presets::arm_windows_point_to_point>,
                                       &windows_over<&presets::arm_windows_via_point>,
                                       &windows_over<&presets::arm_windows_path_comparison>,
                                       &windows_over<&presets::arm_windows_velocity_kinematics>};

static_assert(offered_scenarios.size() == scenario_count);

scene::preset_registry::factory robot_factory(std::filesystem::path named, std::filesystem::path package_root, documents mine, composed_capabilities composed,
                                              std::shared_ptr<write_back> through, composer_factory windows)
{
    return [named = std::move(named), root = std::move(package_root), mine = std::move(mine), composed = std::move(composed), through = std::move(through),
            windows](const scene::preset_site &site)
    {
        const config::binding bound         = machine_binding(named, mine);
        const config::outcome carried       = config::load_or_defaults(bound);
        const presets::arm_scenario machine = read_machine(carried.values, root);
        opened_scenario opened              = windows(machine, scenario_documents{bound, carried.values, machine_screw_table(carried.values, mine)}, composed.motions);

        std::shared_ptr<scene::preset> built = presets::arm_preset(site, composed.arm, composed.shapes, composed.motions, machine, opened.composed);
        if(built != nullptr && through != nullptr)
            through->composing(std::move(opened.announced), std::move(opened.carried));

        return built;
    };
}

}

void report_composed_capabilities()
{
    const composed_capabilities composed = bound_capabilities();

    scene::report_default_slots(composed_views(composed));
}

std::vector<std::string> register_arm_presets(const std::shared_ptr<scene::preset_registry> &registry, const config::document &values, const documents &mine,
                                              const std::filesystem::path &urdf_path, const std::shared_ptr<write_back> &through)
{
    const composed_capabilities composed = bound_capabilities();

    std::vector<std::string> registered;
    for(const std::string &key : preset_keys(values))
    {
        const std::optional<offered_document> offered = offered_preset(values, key, mine);
        if(!offered)
            continue;

        const std::string named = preset_name(offered->carried);
        if(named.empty())
        {
            spdlog::error(std::format("The preset {} is not offered: its document states no name", key));

            continue;
        }

        // The registry assigns into a map, so a second registration under one name would replace the
        // first and leave one entry where two were meant.
        if(registry->load_preset(named) != nullptr)
        {
            spdlog::error(std::format("The preset {} is not offered: the name {} is already carried", key, named));

            continue;
        }

        registered.push_back(named);
        registry->register_preset(named, robot_factory(offered->bound.at.given, urdf_path, mine, composed, through, offered_scenarios[preset_scenario(offered->carried)]));
    }

    return registered;
}

}
