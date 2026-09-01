#include "praxis/presets/arrangements.h"
#include "praxis/presets/arrangement_scenarios.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"

#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/configuration.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/binding.h"

#include "praxis/compat/expected.h"

#include <spdlog/spdlog.h>

#include <span>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>

namespace praxis::presets {

namespace {

constexpr const char *preset_name_key     = "preset/name";
constexpr const char *preset_scenario_key = "preset/scenario";

// The arrangement's own document is located and read where the arrangement is composed, and its
// values are handed to the composer, which owns what that document means for the scenario it builds.
// The name the route is asked about is the one the caller addressed the document by.
scene::preset_registry::factory arrangement_factory(config::location registered, document_route located, composed_route announced, arrangement_composer compose)
{
    return [registered = std::move(registered), located = std::move(located), announced = std::move(announced), compose = std::move(compose)](const scene::preset_site &site)
    {
        const config::binding bound{arrangement_keyspace(), located ? located(registered.given) : registered, config::expectation::partial};
        const config::outcome carried = config::load_or_defaults(bound);

        std::shared_ptr<scene::preset> built = compose(site, carried.values);
        if(built != nullptr && announced)
            announced(bound, carried.values);

        return built;
    };
}

std::string read_name(const config::document &values)
{
    const expected<std::string, config::error> read = values.text(preset_name_key);

    return read ? read.value() : std::string();
}

arrangement_scenario read_scenario(const config::document &values)
{
    const expected<std::string, config::error> read = values.text(preset_scenario_key);

    const std::span<const char *const> labels = arrangement_scenario_labels();
    for(std::size_t option = 0u; read && option < labels.size(); ++option)
        if(read.value() == labels[option])
            return static_cast<arrangement_scenario>(option);

    return arrangement_scenario::euler_pose;
}

std::nullopt_t not_offered(const config::location &at, const std::string &why)
{
    spdlog::error(std::format("The document {} offers no preset: {}", at.resolved.string(), why));

    return std::nullopt;
}

// A document offers a preset by stating a name no document read before it has taken. The registry
// assigns into a map, so a second registration under one name would replace the first and leave one
// entry where two were meant.
std::optional<std::string> offered_name(scene::preset_registry &registry, const config::location &at, const config::outcome &read)
{
    if(read.failure)
        return not_offered(at, read.failure->message);

    const std::string named = read_name(read.values);
    if(named.empty())
        return not_offered(at, "it states no name");

    if(registry.load_preset(named) != nullptr)
        return not_offered(at, std::format("the name {} is already carried", named));

    return named;
}

}

config::declaration arrangement_keyspace()
{
    config::declaration shape("arrangement");
    shape.group("preset");
    shape.field(preset_name_key, config::field_kind::text, std::string());

    const std::span<const char *const> labels = arrangement_scenario_labels();
    shape.choice(preset_scenario_key, std::vector<std::string>(labels.begin(), labels.end()), labels.front());

    rigid_motion::declare_arrangement(shape, arrangement_path);

    return shape;
}

config::binding arrangement_binding(const std::filesystem::path &named, const std::filesystem::path &beside)
{
    return config::binding{arrangement_keyspace(), config::resolve(named, beside), config::expectation::partial};
}

std::vector<std::string> register_arrangements(const std::shared_ptr<scene::preset_registry> &registry, std::span<const config::location> documents, document_route located,
                                               composed_route announced)
{
    return register_arrangements(registry, documents, std::move(located), std::move(announced), rigid_motion::baseline());
}

std::vector<std::string> register_arrangements(const std::shared_ptr<scene::preset_registry> &registry, std::span<const config::location> documents, document_route located,
                                               composed_route announced, const rigid_motion::capabilities &motions)
{
    std::vector<std::string> registered;
    for(const config::location &at : documents)
    {
        const config::binding bound{arrangement_keyspace(), at, config::expectation::partial};
        const config::outcome read = config::load_or_defaults(bound);

        const std::optional<std::string> named = offered_name(*registry, at, read);
        if(!named)
            continue;

        registered.push_back(*named);
        registry->register_preset(*named, arrangement_factory(at, located, announced, composer_for(read_scenario(read.values), motions)));
    }

    return registered;
}

}
