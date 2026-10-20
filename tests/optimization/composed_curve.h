#ifndef HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_COMPOSED_CURVE_H
#define HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_COMPOSED_CURVE_H

#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/strand.h"
#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <optional>

namespace praxis::fixture {

using namespace praxis::optimization;

inline double cubic_value(double x)
{
    return x * x * x - 2.0 * x - 5.0;
}

inline double cubic_slope(double x)
{
    return 3.0 * x * x - 2.0;
}

inline registered_curve cubic()
{
    return registered_curve("cubic", &cubic_value, &cubic_slope);
}

inline scenario_settings opening(std::string selected = "cubic")
{
    return scenario_settings{std::move(selected), interval{-4.0, 4.0}, 3.0, 70u, 1.0e-10};
}

// A scene is created headlessly and the scheduler services every strand inline, so a drain runs
// whatever the composition posted to its work strand.
struct staged
{
    explicit staged(scheduler::scheduler &loop)
            : scene(threepp::Scene::create())
            , site{*scene,
                   loop.main_strand(),
                   *loop.make_strand(),
                   [](std::string) {},
                   [](const std::shared_ptr<scene::imgui_window> &) {},
                   [](const std::shared_ptr<scene::imgui_window> &) {},
                   {}}
    {
    }

    std::shared_ptr<threepp::Scene> scene;
    scene::preset_site site;
};

struct opened
{
    std::shared_ptr<scene::preset> composed;
    std::optional<scenario_window_inputs> inputs;
};

// The composer keeps what it was offered, so a case reads the publication whether or not it composed
// a window.
inline opened open_with(const scene::preset_site &site, std::vector<registered_curve> curves, const capabilities &solving, const scenario_settings &settings, bool graphed = true)
{
    opened taken;
    const scenario_window_composer graphing = [&taken, graphed](const scenario_window_inputs &offered)
    {
        taken.inputs = offered;
        if(!graphed)
            return std::vector<std::shared_ptr<scene::imgui_window>>{};

        return std::vector<std::shared_ptr<scene::imgui_window>>{curve_graph(offered, "Graph")};
    };

    taken.composed = compose_curve_scenario(site, std::move(curves), solving, settings, scenario_composition{graphing});

    return taken;
}

inline std::shared_ptr<const scenario_snapshot> settled(scheduler::scheduler &loop, const opened &taken)
{
    REQUIRE(taken.composed != nullptr);
    REQUIRE(taken.composed->initialize().has_value());
    REQUIRE(loop.drain().has_value());
    REQUIRE(taken.inputs.has_value());

    return taken.inputs->seen.read();
}

}

#endif
