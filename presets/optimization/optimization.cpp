#include "optimization_keys.h"

#include "praxis/presets/routes.h"
#include "praxis/presets/optimization.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/compose_scenario.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"

#include "praxis/compat/expected.h"

#include <spdlog/spdlog.h>

#include <span>
#include <array>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>

namespace praxis::presets {

namespace {

// In the enumeration's own order, which is what indexing the composer table below by an enumerator
// relies on.
constexpr std::array<const char *, 1> scenario_spellings{"curve"};

// The residual |g(x)| a curve's solve stops below.
constexpr double curve_tolerance = 1.0e-10;

std::vector<std::shared_ptr<scene::imgui_window>> curve_windows(const optimization::scenario_window_inputs &inputs)
{
    return {optimization::curve_graph(inputs, "Graph")};
}

optimization_composer curve_composer(const optimization::capabilities &solving, std::vector<optimization::registered_curve> curves)
{
    return [solving, curves = std::move(curves)](const scene::preset_site &site, const config::document &carried)
    { return optimization::compose_curve_scenario(site, curves, solving, keys::curve_settings(carried, curve_tolerance), optimization::scenario_composition{&curve_windows}); };
}

using composer_factory = optimization_composer (*)(const optimization::capabilities &, std::vector<optimization::registered_curve>);

constexpr std::array<composer_factory, 1> offered_scenarios{&curve_composer};

// The scenario's own document is located and read where it is composed, so a document saved since
// the registration is the one opened.
scene::preset_registry::factory optimization_factory(config::location registered, document_route located, composed_route announced, optimization_composer compose)
{
    return [registered = std::move(registered), located = std::move(located), announced = std::move(announced), compose = std::move(compose)](const scene::preset_site &site)
    {
        const config::binding bound{optimization_keyspace(), located ? located(registered.given) : registered, config::expectation::partial};
        const config::outcome carried = config::load_or_defaults(bound);

        std::shared_ptr<scene::preset> built = compose(site, carried.values);
        if(built != nullptr && announced)
            announced(bound, carried.values);

        return built;
    };
}

optimization_scenario read_scenario(const config::document &values)
{
    const expected<std::string, config::error> read = values.text(keys::preset_scenario);

    const std::span<const char *const> labels = optimization_scenario_labels();
    for(std::size_t option = 0u; read && option < labels.size(); ++option)
        if(read.value() == labels[option])
            return static_cast<optimization_scenario>(option);

    return optimization_scenario::curve;
}

std::nullopt_t not_offered(const config::location &at, const std::string &why)
{
    spdlog::error(std::format("The document {} offers no preset: {}", at.resolved.string(), why));

    return std::nullopt;
}

// The registry assigns into a map, so a second registration under one name would replace the first.
std::optional<std::string> offered_name(scene::preset_registry &registry, const config::location &at, const config::outcome &read)
{
    if(read.failure)
        return not_offered(at, read.failure->message);

    const expected<std::string, config::error> named = read.values.text(keys::preset_name);
    if(!named || named.value().empty())
        return not_offered(at, "it states no name");

    if(registry.load_preset(named.value()) != nullptr)
        return not_offered(at, std::format("the name {} is already carried", named.value()));

    return named.value();
}

}

std::span<const char *const> optimization_scenario_labels()
{
    return scenario_spellings;
}

optimization_composer composer_for(optimization_scenario scenario, const optimization::capabilities &solving, std::vector<optimization::registered_curve> curves)
{
    return offered_scenarios.at(static_cast<std::size_t>(scenario))(solving, std::move(curves));
}

std::vector<std::string> register_optimization(const std::shared_ptr<scene::preset_registry> &registry, std::span<const config::location> documents, document_route located,
                                               composed_route announced, const optimization::capabilities &solving, std::span<const optimization::registered_curve> curves)
{
    const std::vector<optimization::registered_curve> offered(curves.begin(), curves.end());

    std::vector<std::string> registered;
    for(const config::location &at : documents)
    {
        const config::outcome read = config::load_or_defaults(config::binding{optimization_keyspace(), at, config::expectation::partial});

        const std::optional<std::string> named = offered_name(*registry, at, read);
        if(!named)
            continue;

        registered.push_back(*named);
        registry->register_preset(*named, optimization_factory(at, located, announced, composer_for(read_scenario(read.values), solving, offered)));
    }

    return registered;
}

}
