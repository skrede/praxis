#include "registered_list.h"

#include "praxis/optimization/surface_stencil.h"
#include "praxis/optimization/compose_scenario.h"

#include "praxis/scene/stencil.h"

#include <spdlog/spdlog.h>

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
    if(std::find(registered.begin(), registered.end(), opened.selected) == registered.end())
        return "it opens at '" + opened.selected + "', which is not registered (registered: " + listed(registered) + ")";
    if(opened.ranges.size() != dimension || static_cast<std::size_t>(opened.seed.size()) != dimension)
        return std::format("it opens over {} ranges from a seed of {} coordinates, and its functions take {}", opened.ranges.size(), opened.seed.size(), dimension);

    return std::nullopt;
}

// The gate is neither copyable nor movable, so it is built where it stands, and its last share is
// handed to the preset's release callable: freeing it earlier would let a run the gate has already
// posted reach storage that is gone.
template<typename registered, typename drawing>
std::shared_ptr<scene::preset> composed_over(const scene::preset_site &site, std::vector<registered> functions, const capabilities &solving, const scenario_settings &opened,
                                             const scenario_composition &composed, const char *kind, std::shared_ptr<surface_view> view, drawing drawn)
{
    const std::size_t dimension                     = std::is_same_v<registered, registered_curve> ? 1u : 2u;
    const std::vector<std::string> registered_names = names_of(functions);
    if(const std::optional<std::string> why = unopenable(registered_names, opened, dimension))
        return refused(kind, *why);

    auto published = std::make_shared<scenario_publisher>();
    auto owned     = std::make_shared<owned_scenario>(site.work, std::move(functions), solving, opened, published, composed.checks_derivative);
    if(!owned->with([](scenario_state &state) { state.run(); }))
        return refused(kind, "its work strand admits no run");

    const scenario_window_inputs built{published->reader(), dimension, composed.checks_derivative, opened, registered_names, owned, std::move(view)};
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
    return composed_over(site, std::move(curves), solving, opened, composed, "curve", nullptr, [](const scenario_window_inputs &) { return std::make_shared<empty_stencil>(); });
}

std::shared_ptr<scene::preset> compose_surface_scenario(const scene::preset_site &site, std::vector<registered_surface> surfaces, const capabilities &solving,
                                                        const scenario_settings &opened, const scenario_composition &composed)
{
    return composed_over(site, std::move(surfaces), solving, opened, composed, "surface", std::make_shared<surface_view>(),
                         [&site](const scenario_window_inputs &built) { return std::make_shared<surface_stencil>(site.scene, built.seen, built.drawn); });
}

}
