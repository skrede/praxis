#include "registered_list.h"
#include "scenario_defaults.h"
#include "scenario_admission.h"

#include "praxis/optimization/settings_keys.h"
#include "praxis/optimization/surface_stencil.h"
#include "praxis/optimization/compose_scenario.h"

#include "praxis/scene/stencil.h"
#include "praxis/scene/key_window.h"

#include <threepp/math/Vector3.hpp>

#include <spdlog/spdlog.h>

#include <cmath>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>
#include <type_traits>

namespace praxis::optimization {

namespace {

// A function of one variable is drawn in a plot, so the scene is given nothing.
class empty_stencil : public scene::stencil
{
public:
    expected<void, refusal> initialize() override
    {
        return {};
    }

    void tear_down() override
    {
    }

    void render() const override
    {
    }
};

std::string listed(const std::vector<std::string> &names)
{
    std::string joined;
    for(const std::string &name : names)
        joined += (joined.empty() ? "'" : ", '") + name + "'";

    return joined.empty() ? "none" : joined;
}

std::shared_ptr<scene::preset> refused(const char *kind, const std::string &why)
{
    spdlog::error("praxis: the {} scenario is not composed: {}", kind, why);

    return nullptr;
}

std::optional<std::string> unopenable(const std::vector<std::string> &registered, const scenario_settings &opened, std::size_t dimension)
{
    if(opened.typed.source != function_source::manual && std::find(registered.begin(), registered.end(), opened.selected) == registered.end())
        return std::format("it opens at '{}', which is not registered for the {} form (registered: {})", opened.selected,
                           form_spellings.at(static_cast<std::size_t>(opened.objective.form)), listed(registered));
    if(opened.ranges.size() != dimension || static_cast<std::size_t>(opened.seed.size()) != dimension)
        return std::format("it opens over {} ranges from a seed of {} coordinates, and its functions take {}", opened.ranges.size(), opened.seed.size(), dimension);
    if(!drawable(opened.ranges, dimension))
        return "it opens over a range that is not finite or whose minimum does not lie below its maximum";
    if(!opened.seed.allFinite())
        return "it opens from a seed that is not finite";

    return std::nullopt;
}

scene::camera_view surface_opening_view()
{
    const double level = opening_distance * std::cos(opening_elevation_radians);
    const threepp::Vector3 position(static_cast<float>(level * std::sin(opening_azimuth_radians)), static_cast<float>(opening_distance * std::sin(opening_elevation_radians)),
                                    static_cast<float>(level * std::cos(opening_azimuth_radians)));

    return scene::camera_view{position, threepp::Vector3(0.f, 0.f, 0.f)};
}

// The gate is neither copyable nor movable, so it is built where it stands, and its last share is
// handed to the preset's release callable: freeing it earlier would let a run the gate has already
// posted reach storage that is gone.
template<typename registered, typename drawing>
std::shared_ptr<scene::preset> composed_over(const scene::preset_site &site, std::vector<registered> functions, const capabilities &solving, scenario_settings opened,
                                             const scenario_composition &composed, const char *kind, drawing drawn)
{
    const std::size_t dimension                     = std::is_same_v<registered, registered_curve> ? 1u : 2u;
    const std::vector<std::string> registered_names = names_of(functions);
    opened.typed                                    = lettered(std::move(opened.typed), dimension);
    if(const std::optional<std::string> why = unopenable(registered_names, opened, dimension))
        return refused(kind, *why);

    auto published = std::make_shared<scenario_publisher>();
    auto owned     = std::make_shared<owned_scenario>(site.work, std::move(functions), solving, opened, published, composed.checks_derivative);
    if(!owned->with([](scenario_state &state) { state.run(); }))
        return refused(kind, "its work strand admits no run");

    auto keyed = std::make_shared<std::vector<scene::key_entry>>();
    const scenario_window_inputs built{published->reader(), dimension, composed.checks_derivative, opened, registered_names, owned, std::make_shared<scenario_view>(), std::move(keyed)};
    std::vector<std::shared_ptr<scene::imgui_window>> windows = composed.windows ? composed.windows(built) : std::vector<std::shared_ptr<scene::imgui_window>>{};

    auto preset        = std::make_shared<scene::preset>(drawn(built), std::move(windows), site.add_window, site.remove_window);
    preset->work       = site.work;
    preset->release_cb = [owned]() mutable { owned.reset(); };

    return preset;
}

}

std::shared_ptr<scene::preset> compose_curve_scenario(const scene::preset_site &site, std::vector<registered_curve> curves, const capabilities &solving, const scenario_settings &opened,
                                                      const scenario_composition &composed)
{
    return composed_over(site, offered_in(std::move(curves), opened.objective.form), solving, opened, composed, "curve",
                         [](const scenario_window_inputs &) { return std::make_shared<empty_stencil>(); });
}

std::shared_ptr<scene::preset> compose_surface_scenario(const scene::preset_site &site, std::vector<registered_surface> surfaces, const capabilities &solving,
                                                        const scenario_settings &opened, const scenario_composition &composed)
{
    const auto drawing                    = [&site](const scenario_window_inputs &built) { return std::make_shared<surface_stencil>(site.scene, built.seen, built.drawn, built.keyed); };
    std::shared_ptr<scene::preset> preset = composed_over(site, offered_in(std::move(surfaces), opened.objective.form), solving, opened, composed, "surface", drawing);
    if(preset != nullptr)
        preset->opening_view = surface_opening_view();

    return preset;
}

}
