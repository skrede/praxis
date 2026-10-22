#ifndef HPP_GUARD_PRAXIS_TESTS_EXAMPLES_OFFERED_OPTIMIZATION_H
#define HPP_GUARD_PRAXIS_TESTS_EXAMPLES_OFFERED_OPTIMIZATION_H

#include "scratch_documents.h"

#include "imgui_frame.h"

#include "demo_offered.h"
#include "demo_documents.h"
#include "demo_write_back.h"
#include "demo_configuration.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <filesystem>

#ifndef PRAXIS_SHIPPED_OPTIMIZATION_DIR
    #define PRAXIS_SHIPPED_OPTIMIZATION_DIR ""
#endif

namespace offered_optimization {

inline praxis::config::location shipped_list()
{
    return praxis::config::resolve(std::filesystem::path(PRAXIS_SHIPPED_OPTIMIZATION_DIR) / "praxis-optimization.xml", PRAXIS_SHIPPED_OPTIMIZATION_DIR);
}

inline praxis::config::document demonstration()
{
    const praxis::config::outcome answered = praxis::config::load_or_defaults(praxis::demo::demonstration_keyspace(), shipped_list());
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
}

// The registry the application builds over the shipped documents, with its own registrations, a
// scratch directory standing in for the one it writes to.
struct offered
{
    explicit offered(const scratch_documents::scratch &where)
            : mine(PRAXIS_SHIPPED_OPTIMIZATION_DIR, where.state())
            , preferences{praxis::demo::preferences_keyspace(), praxis::config::resolve("praxis-preferences.xml", where.state()), praxis::config::expectation::partial}
            , writing(std::make_shared<praxis::demo::write_back>(praxis::config::binding{praxis::demo::demonstration_keyspace(), shipped_list(), praxis::config::expectation::partial},
                                                                 demonstration(), preferences, praxis::config::load_or_defaults(preferences).values, mine))
            , registry(std::make_shared<praxis::scene::preset_registry>())
            , names(praxis::demo::register_offered(registry, mine, demonstration(), {}, writing))
    {
    }

    praxis::demo::documents mine;
    praxis::config::binding preferences;
    std::shared_ptr<praxis::demo::write_back> writing;
    std::shared_ptr<praxis::scene::preset_registry> registry;
    std::vector<std::string> names;
};

// Composes the named preset on a scene of its own, hands it to `visit` with the scheduler its strands
// run on and the scene, then retires it.
template<typename visiting>
void on_own_scene(praxis::scene::preset_registry &registry, const std::string &named, visiting &&visit)
{
    praxis::scheduler::scheduler loop(praxis::scheduler::inline_workers);
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    const praxis::scene::window_route nowhere    = [](const std::shared_ptr<praxis::scene::imgui_window> &) {};
    const praxis::scene::preset_site site{*target, loop.main_strand(), *loop.make_strand(), [](std::string) {}, nowhere, nowhere, {}};

    std::shared_ptr<praxis::scene::preset> composed = registry.load_preset(named)(site);
    REQUIRE(composed != nullptr);
    REQUIRE(composed->initialize().has_value());
    std::forward<visiting>(visit)(*composed, loop, *target);

    composed->tear_down();
    REQUIRE(loop.retire_strand(composed->work, std::move(composed->release_cb)).has_value());
    REQUIRE(loop.drain().has_value());
}

// The text the Solve window draws once the composition's work strand has run.
inline std::string solve_text(praxis::scene::preset_registry &registry, const std::string &named)
{
    std::string drawn;
    on_own_scene(registry, named,
                 [&drawn](praxis::scene::preset &composed, praxis::scheduler::scheduler &loop, threepp::Scene &)
                 {
                     REQUIRE(loop.drain().has_value());
                     const auto solve =
                             std::ranges::find_if(composed.windows, [](const std::shared_ptr<praxis::scene::imgui_window> &window) { return window->display_name() == "Solve"; });
                     REQUIRE(solve != composed.windows.end());
                     praxis::tests::imgui_frame frames;
                     frames.draw(
                             [&solve, &drawn]
                             {
                                 ImGui::LogToBuffer();
                                 (*solve)->render();
                                 drawn = GImGui->LogBuffer.c_str();
                                 ImGui::LogFinish();
                             });
                 });

    return drawn;
}

}

#endif
