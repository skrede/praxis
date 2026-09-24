#include "arm_keys.h"
#include "arm_composers.h"

#include "praxis/presets/arm.h"
#include "praxis/presets/screw_table.h"
#include "praxis/presets/arm_scenarios.h"
#include "praxis/presets/arm_registration.h"

#include "praxis/manipulator/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"

#include <spdlog/spdlog.h>

#include <span>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <filesystem>

namespace praxis::presets {

namespace {

struct arm_capabilities
{
    manipulator::capabilities arm;
    trajectory::capabilities shapes;
    rigid_motion::capabilities motions;
};

// Where one arm keeps a chain somebody supplied, asked of the caller's route by the name the arm's
// own document carried. An arm naming no such document keeps nothing, and the scenario reading this
// opens at a chain nobody supplied.
std::optional<config::binding> kept_chain(const config::document &values, const config::location &registered, const document_route &located)
{
    const std::string named = keys::text_at(values, keys::screw_table_key);
    if(named.empty())
        return std::nullopt;

    const config::location at = located ? located(named) : config::resolve(named, registered.resolved.parent_path());

    return config::binding{screw_table_keyspace(), at, config::expectation::partial};
}

// The arm's own document is located and read where the scenario is composed, and the roots it
// resolves its description against are owned here rather than borrowed, because a span outlives
// nothing.
scene::preset_registry::factory arm_factory(config::location registered, std::vector<std::filesystem::path> roots, document_route located, composed_route announced,
                                            arm_capabilities composed, composer_factory windows)
{
    return [registered = std::move(registered), roots = std::move(roots), located = std::move(located), announced = std::move(announced), composed = std::move(composed),
            windows](const scene::preset_site &site)
    {
        const config::binding bound{arm_keyspace(), located ? located(registered.given) : registered, config::expectation::partial};
        const config::outcome carried = config::load_or_defaults(bound);
        const arm_scenario machine    = read_arm(carried.values, roots);

        const scenario_documents documents{bound, carried.values, kept_chain(carried.values, bound.at, located)};
        const opened_scenario opened = windows(machine, documents, composed.motions);

        std::shared_ptr<scene::preset> built = arm_preset(site, composed.arm, composed.shapes, composed.motions, machine, opened.composed);
        if(built != nullptr && announced)
            announced(opened.announced, opened.carried);

        return built;
    };
}

std::nullopt_t not_offered(const config::location &at, const std::string &why)
{
    spdlog::error(std::format("The document {} offers no preset: {}", at.resolved.string(), why));

    return std::nullopt;
}

// A document offers a preset by loading at all, by stating a name no document read before it has
// taken, and by naming a description. The registry assigns into a map, so a second registration
// under one name would replace the first and leave one entry where two were meant.
std::optional<std::string> offered_name(scene::preset_registry &registry, const config::location &at, const config::outcome &read)
{
    if(read.failure)
        return not_offered(at, read.failure->message);

    const std::string named = keys::text_at(read.values, keys::preset_name_key);
    if(named.empty())
        return not_offered(at, "it states no name");

    if(registry.load_preset(named) != nullptr)
        return not_offered(at, std::format("the name {} is already carried", named));

    if(keys::text_at(read.values, keys::description_path_key).empty())
        return not_offered(at, "it names no description");

    return named;
}

std::optional<arm_scenario_kind> offered_scenario(const config::location &at, const config::document &values)
{
    const std::optional<arm_scenario_kind> named = arm_scenario_named(values);
    if(!named)
        return not_offered(at, std::format("it names the scenario {}, which is not offered", keys::text_at(values, keys::preset_scenario_key)));

    return named;
}

}

std::vector<std::string> register_arms(const std::shared_ptr<scene::preset_registry> &registry, std::span<const config::location> documents,
                                       std::span<const std::filesystem::path> roots, document_route located, composed_route announced)
{
    return register_arms(registry, documents, roots, std::move(located), std::move(announced), manipulator::baseline(), trajectory::baseline(), rigid_motion::baseline());
}

std::vector<std::string> register_arms(const std::shared_ptr<scene::preset_registry> &registry, std::span<const config::location> documents,
                                       std::span<const std::filesystem::path> roots, document_route located, composed_route announced, const manipulator::capabilities &arm,
                                       const trajectory::capabilities &shapes, const rigid_motion::capabilities &motions)
{
    if(registry == nullptr)
    {
        spdlog::error("praxis: no arm was registered, because no registry was supplied");
        return {};
    }

    const std::vector<std::filesystem::path> searched(roots.begin(), roots.end());

    std::vector<std::string> registered;
    for(const config::location &at : documents)
    {
        const config::binding bound{arm_keyspace(), at, config::expectation::partial};
        const config::outcome read = config::load_or_defaults(bound);

        const std::optional<std::string> named          = offered_name(*registry, at, read);
        const std::optional<arm_scenario_kind> scenario = named ? offered_scenario(at, read.values) : std::nullopt;
        if(!scenario)
            continue;

        registered.push_back(*named);
        registry->register_preset(*named, arm_factory(at, searched, located, announced, arm_capabilities{arm, shapes, motions}, composer_for(*scenario)));
    }

    return registered;
}

}
