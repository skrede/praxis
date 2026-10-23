#include "optimization_keys.h"
#include "window_placements.h"

#include "praxis/presets/routes.h"
#include "praxis/presets/optimization.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/solve_window.h"
#include "praxis/optimization/display_window.h"
#include "praxis/optimization/function_window.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/linearization_window.h"

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
#include <string_view>

namespace praxis::presets {

namespace {

struct offered_functions
{
    std::vector<optimization::registered_curve> curves;
    std::vector<optimization::registered_surface> surfaces;
};

// In the enumeration's own order, which is what indexing the composer table below by an enumerator
// relies on.
constexpr std::array<const char *, 2> scenario_spellings{"curve", "surface"};

// The GUI library keys a window by its whole name and shows only what precedes "##", so the two
// scenarios' windows share their titles and each opens at its own scenario's placement.
std::string scenario_name(std::string_view title, optimization_scenario scenario)
{
    return std::format("{}##{}", title, scenario_spellings.at(static_cast<std::size_t>(scenario)));
}

std::string_view shown_title(const std::string &name)
{
    return std::string_view(name).substr(0, name.find("##"));
}

std::vector<std::shared_ptr<scene::imgui_window>> placed(std::vector<std::shared_ptr<scene::imgui_window>> windows, std::size_t dimension)
{
    for(const std::shared_ptr<scene::imgui_window> &window : windows)
        if(const std::optional<scene::opening_placement> at = scenario_window_placement(shown_title(window->display_name()), dimension))
            window->open_at(*at);

    return windows;
}

std::vector<std::shared_ptr<scene::imgui_window>> curve_windows(const optimization::scenario_window_inputs &inputs)
{
    const auto named = [](std::string_view title) { return scenario_name(title, optimization_scenario::curve); };
    return placed({std::make_shared<optimization::function_window>(named("Function"), inputs), optimization::curve_graph(inputs, named("Graph")),
                   std::make_shared<optimization::display_window>(named("Display"), inputs), std::make_shared<optimization::solve_window>(named("Solve"), inputs),
                   std::make_shared<optimization::linearization_window>(named("Linearization"), inputs)},
                  1u);
}

std::vector<std::shared_ptr<scene::imgui_window>> surface_windows(const optimization::scenario_window_inputs &inputs)
{
    const auto named = [](std::string_view title) { return scenario_name(title, optimization_scenario::surface); };
    return placed({std::make_shared<optimization::function_window>(named("Function"), inputs), std::make_shared<optimization::display_window>(named("Display"), inputs),
                   std::make_shared<optimization::solve_window>(named("Solve"), inputs), std::make_shared<optimization::linearization_window>(named("Linearization"), inputs)},
                  2u);
}

optimization_composer curve_composer(const optimization::capabilities &solving, offered_functions offered, bool checks_derivative)
{
    return [solving, curves = std::move(offered.curves), checks_derivative](const scene::preset_site &site, const config::document &carried)
    { return optimization::compose_curve_scenario(site, curves, solving, keys::opened_settings(carried, 1u), optimization::scenario_composition{&curve_windows, checks_derivative}); };
}

optimization_composer surface_composer(const optimization::capabilities &solving, offered_functions offered, bool checks_derivative)
{
    return [solving, surfaces = std::move(offered.surfaces), checks_derivative](const scene::preset_site &site, const config::document &carried)
    {
        return optimization::compose_surface_scenario(site, surfaces, solving, keys::opened_settings(carried, 2u),
                                                      optimization::scenario_composition{&surface_windows, checks_derivative});
    };
}

using composer_factory = optimization_composer (*)(const optimization::capabilities &, offered_functions, bool);

constexpr std::array<composer_factory, 2> offered_scenarios{&curve_composer, &surface_composer};

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

// The registry replaces the factory already registered under a name, so a name already carried is left out here rather than replacing it.
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

optimization_composer composer_for(optimization_scenario scenario, const optimization::capabilities &solving, std::vector<optimization::registered_curve> curves,
                                   std::vector<optimization::registered_surface> surfaces, bool checks_derivative)
{
    return offered_scenarios.at(static_cast<std::size_t>(scenario))(solving, offered_functions{std::move(curves), std::move(surfaces)}, checks_derivative);
}

std::vector<std::string> register_optimization(const std::shared_ptr<scene::preset_registry> &registry, std::span<const config::location> documents, document_route located,
                                               composed_route announced, const optimization::capabilities &solving, std::span<const optimization::registered_curve> curves,
                                               std::span<const optimization::registered_surface> surfaces, bool checks_derivative)
{
    const std::vector<optimization::registered_curve> offered(curves.begin(), curves.end());
    const std::vector<optimization::registered_surface> offered_surfaces(surfaces.begin(), surfaces.end());

    std::vector<std::string> registered;
    for(const config::location &at : documents)
    {
        const config::outcome read = config::load_or_defaults(config::binding{optimization_keyspace(), at, config::expectation::partial});

        const std::optional<std::string> named = offered_name(*registry, at, read);
        if(!named)
            continue;

        registered.push_back(*named);
        registry->register_preset(*named, optimization_factory(at, located, announced, composer_for(read_scenario(read.values), solving, offered, offered_surfaces, checks_derivative)));
    }

    return registered;
}

}
