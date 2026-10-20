#include "demo_offered.h"
#include "demo_documents.h"
#include "demo_write_back.h"
#include "demo_configuration.h"
#include "scratch_documents.h"

#include "imgui_frame.h"

#include "praxis/presets/optimization.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/objects/Mesh.hpp>

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <filesystem>

#ifndef PRAXIS_SHIPPED_OPTIMIZATION_DIR
    #define PRAXIS_SHIPPED_OPTIMIZATION_DIR ""
#endif

using namespace praxis;
using namespace scratch_documents;

namespace {

config::location shipped_list()
{
    return config::resolve(std::filesystem::path(PRAXIS_SHIPPED_OPTIMIZATION_DIR) / "praxis-optimization.xml", PRAXIS_SHIPPED_OPTIMIZATION_DIR);
}

config::document demonstration()
{
    const config::outcome answered = config::load_or_defaults(demo::demonstration_keyspace(), shipped_list());
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
}

// The registry the application builds over the shipped documents, with its own registrations, a
// scratch directory standing in for the one it writes to.
struct offered
{
    explicit offered(const scratch &where)
            : mine(PRAXIS_SHIPPED_OPTIMIZATION_DIR, where.state())
            , preferences{demo::preferences_keyspace(), config::resolve("praxis-preferences.xml", where.state()), config::expectation::partial}
            , writing(std::make_shared<demo::write_back>(config::binding{demo::demonstration_keyspace(), shipped_list(), config::expectation::partial}, demonstration(), preferences,
                                                         config::load_or_defaults(preferences).values, mine))
            , registry(std::make_shared<scene::preset_registry>())
            , names(demo::register_offered(registry, mine, demonstration(), {}, writing))
    {
    }

    demo::documents mine;
    config::binding preferences;
    std::shared_ptr<demo::write_back> writing;
    std::shared_ptr<scene::preset_registry> registry;
    std::vector<std::string> names;
};

// What one composition draws before and after its work strand has run, and the windows it opened.
struct drawn
{
    std::vector<std::string> windows;
    int before;
    int after;
    int field;
};

int vertices_of(const std::vector<std::shared_ptr<scene::imgui_window>> &windows)
{
    tests::imgui_frame frames;
    frames.draw(
            [&windows]
            {
                for(const std::shared_ptr<scene::imgui_window> &window : windows)
                    window->render();
            });

    return frames.vertices();
}

drawn composed_and_drawn(scene::preset_registry &registry, const std::string &named)
{
    scheduler::scheduler loop(scheduler::inline_workers);
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    const scene::window_route nowhere            = [](const std::shared_ptr<scene::imgui_window> &) {};
    const scene::preset_site site{*target, loop.main_strand(), *loop.make_strand(), [](std::string) {}, nowhere, nowhere, {}};

    std::shared_ptr<scene::preset> composed = registry.load_preset(named)(site);
    REQUIRE(composed != nullptr);
    REQUIRE(composed->initialize().has_value());

    drawn seen{{}, 0, 0, 0};
    for(const std::shared_ptr<scene::imgui_window> &window : composed->windows)
        seen.windows.push_back(window->display_name());
    REQUIRE_FALSE(composed->windows.empty());
    seen.before = vertices_of(composed->windows);
    REQUIRE(loop.drain().has_value());
    seen.after = vertices_of(composed->windows);
    composed->stencil->render();
    if(const auto *field = target->getObjectByName<threepp::Mesh>("surface"))
        seen.field = field->geometry()->drawRange.count;

    composed->tear_down();
    REQUIRE(loop.retire_strand(composed->work, std::move(composed->release_cb)).has_value());
    REQUIRE(loop.drain().has_value());

    return seen;
}

}

TEST_CASE("every shipped optimization document loads and states its scenario", "[examples][optimization]")
{
    const scratch where("shipped-optimization-loads");
    const offered shipped(where);

    const std::vector<config::location> named = demo::preset_locations(demonstration(), shipped.mine);
    REQUIRE(named.size() == 2u);
    for(const config::location &at : named)
    {
        INFO(at.resolved.string());
        const config::outcome read = config::load_or_defaults(presets::optimization_keyspace(), at, config::expectation::partial);
        REQUIRE_FALSE(read.failure.has_value());
        CHECK(read.values.origin_of("preset/scenario").kind == config::origin_kind::source);
        CHECK(read.values.origin_of("function/name").kind == config::origin_kind::source);
    }
}

TEST_CASE("the shipped optimization documents register under the names they state", "[examples][optimization]")
{
    const scratch where("shipped-optimization-names");

    CHECK(offered(where).names == std::vector<std::string>{"Root finding: a cubic", "Root finding: a paraboloid"});
}

TEST_CASE("every shipped optimization preset composes over the demonstration's functions and draws its run", "[examples][optimization]")
{
    const scratch where("shipped-optimization-runs");
    const offered shipped(where);

    REQUIRE(shipped.names.size() == 2u);

    const drawn curve = composed_and_drawn(*shipped.registry, shipped.names[0]);
    CHECK(curve.windows == std::vector<std::string>{"Function", "Graph", "Solve", "Linearization"});
    CHECK(curve.after > curve.before);
    CHECK(curve.field == 0);

    const drawn surface = composed_and_drawn(*shipped.registry, shipped.names[1]);
    CHECK(surface.windows == std::vector<std::string>{"Function", "Graph", "Solve", "Linearization"});
    CHECK(surface.after > surface.before);
    CHECK(surface.field > 0);
}
