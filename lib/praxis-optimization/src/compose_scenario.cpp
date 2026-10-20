#include "praxis/optimization/compose_scenario.h"

#include "praxis/scene/stencil.h"

#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

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

std::vector<std::string> names_of(const std::vector<registered_curve> &curves)
{
    std::vector<std::string> named;
    for(const registered_curve &curve : curves)
        named.push_back(curve.name);

    return named;
}

std::string listed(const std::vector<std::string> &names)
{
    std::string joined;
    for(const std::string &name : names)
        joined += (joined.empty() ? "'" : ", '") + name + "'";

    return joined.empty() ? "none" : joined;
}

std::shared_ptr<scene::preset> refused(const std::string &why)
{
    spdlog::error("praxis: the curve scenario is not composed: {}", why);

    return nullptr;
}

}

// The gate is neither copyable nor movable, so it is built where it stands, and its last share is
// handed to the preset's release callable: freeing it earlier would let a run the gate has already
// posted reach storage that is gone.
std::shared_ptr<scene::preset> compose_curve_scenario(const scene::preset_site &site, std::vector<registered_curve> curves, const capabilities &solving, const scenario_settings &opened,
                                                      const scenario_composition &composed)
{
    const std::vector<std::string> registered = names_of(curves);
    if(std::find(registered.begin(), registered.end(), opened.selected) == registered.end())
        return refused("it opens at '" + opened.selected + "', which is not registered (registered: " + listed(registered) + ")");

    auto published = std::make_shared<scenario_publisher>();
    auto owned     = std::make_shared<owned_scenario>(site.work, std::move(curves), solving, opened, published);
    if(!owned->with([](scenario_state &state) { state.run(); }))
        return refused("its work strand admits no run");

    const scenario_window_inputs built{published->reader(), 1u, registered, owned};
    std::vector<std::shared_ptr<scene::imgui_window>> windows = composed.windows ? composed.windows(built) : std::vector<std::shared_ptr<scene::imgui_window>>{};

    auto preset        = std::make_shared<scene::preset>(std::make_shared<empty_stencil>(), std::move(windows), site.add_window, site.remove_window);
    preset->work       = site.work;
    preset->release_cb = [owned]() mutable { owned.reset(); };

    return preset;
}

}
