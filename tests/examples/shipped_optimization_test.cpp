#include "scratch_documents.h"
#include "offered_optimization.h"

#include "demo_configuration.h"

#include "imgui_frame.h"

#include "praxis/presets/optimization.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"
#include "praxis/config/document.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/objects/Mesh.hpp>

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <string>
#include <vector>

using namespace praxis;
using namespace scratch_documents;
using namespace offered_optimization;

namespace {

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
    drawn seen{{}, 0, 0, 0};
    on_own_scene(registry, named,
                 [&seen](scene::preset &composed, scheduler::scheduler &loop, threepp::Scene &target)
                 {
                     for(const std::shared_ptr<scene::imgui_window> &window : composed.windows)
                         seen.windows.push_back(window->display_name());
                     REQUIRE_FALSE(composed.windows.empty());
                     seen.before = vertices_of(composed.windows);
                     REQUIRE(loop.drain().has_value());
                     seen.after = vertices_of(composed.windows);
                     composed.stencil->render();
                     if(const auto *field = target.getObjectByName<threepp::Mesh>("surface"))
                         seen.field = field->geometry()->drawRange.count;
                 });

    return seen;
}

}

TEST_CASE("every shipped optimization document loads and states its scenario", "[examples][optimization]")
{
    const scratch where("shipped-optimization-loads");
    const offered shipped(where);

    const std::vector<config::location> named = demo::preset_locations(demonstration(), shipped.mine);
    REQUIRE(named.size() == 4u);
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

    CHECK(offered(where).names == std::vector<std::string>{"Root finding: a cubic", "Root finding: a paraboloid", "Minimization: x^4 - 3x^2 + x", "Minimization: (x^2 + 10y^2)/2"});
}

TEST_CASE("every shipped optimization preset composes over the demonstration's functions and draws its run", "[examples][optimization]")
{
    const scratch where("shipped-optimization-runs");
    const offered shipped(where);

    REQUIRE(shipped.names.size() == 4u);

    const drawn curve = composed_and_drawn(*shipped.registry, shipped.names[0]);
    CHECK(curve.windows == std::vector<std::string>{"Function", "Graph", "Display", "Solve", "Linearization"});
    CHECK(curve.after > curve.before);
    CHECK(curve.field == 0);

    const drawn surface = composed_and_drawn(*shipped.registry, shipped.names[1]);
    CHECK(surface.windows == std::vector<std::string>{"Function", "Display", "Solve", "Linearization"});
    CHECK(surface.after > surface.before);
    CHECK(surface.field > 0);
}
