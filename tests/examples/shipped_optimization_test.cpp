#include "scratch_documents.h"
#include "offered_optimization.h"

#include "demo_configuration.h"

#include "imgui_frame.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/settings_keys.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"
#include "praxis/config/document.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/objects/Mesh.hpp>

#include <threepp/scenes/Scene.hpp>

#include <tuple>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>

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
    REQUIRE(named.size() == 6u);
    for(const config::location &at : named)
    {
        INFO(at.resolved.string());
        const config::outcome read = config::load_or_defaults(presets::optimization_keyspace(), at, config::expectation::partial);
        REQUIRE_FALSE(read.failure.has_value());
        CHECK(read.values.origin_of("preset/scenario").kind == config::origin_kind::source);
        const bool registered = read.values.origin_of(optimization::function_name_key).kind == config::origin_kind::source;
        const bool typed      = read.values.origin_of(optimization::expression_key).kind == config::origin_kind::source;
        CHECK(registered != typed);
    }
}

TEST_CASE("the shipped optimization documents register under the names they state", "[examples][optimization]")
{
    const scratch where("shipped-optimization-names");

    CHECK(offered(where).names ==
          std::vector<std::string>{"Root finding: a cubic", "Root finding: a paraboloid", "Minimization: x^4 - 3x^2 + x", "Minimization: (x^2 + 10y^2)/2", "Typed: 2x^3 + 5b",
                                   "Typed: x^2 + ay^2 - 1"});
}

TEST_CASE("every shipped optimization preset composes over the demonstration's functions and draws its run", "[examples][optimization]")
{
    const scratch where("shipped-optimization-runs");
    const offered shipped(where);

    REQUIRE(shipped.names.size() == 6u);

    const drawn curve = composed_and_drawn(*shipped.registry, shipped.names[0]);
    CHECK(curve.windows == std::vector<std::string>{"Function", "Graph", "Display", "Solve", "Linearization"});
    CHECK(curve.after > curve.before);
    CHECK(curve.field == 0);

    const drawn surface = composed_and_drawn(*shipped.registry, shipped.names[1]);
    CHECK(surface.windows == std::vector<std::string>{"Function", "Display", "Solve", "Linearization"});
    CHECK(surface.after > surface.before);
    CHECK(surface.field > 0);
}

TEST_CASE("the shipped typed documents state their expression and one parameter per letter it names", "[examples][optimization]")
{
    for(const auto &[document, expression, letter] : {std::tuple{"curve-typed.xml", "2x^3 + 5b", "b"}, std::tuple{"surface-typed.xml", "x^2 + ay^2 - 1", "a"}})
    {
        INFO(document);
        const config::location at  = config::resolve(document, PRAXIS_SHIPPED_OPTIMIZATION_DIR);
        const config::outcome read = config::load_or_defaults(presets::optimization_keyspace(), at, config::expectation::partial);
        REQUIRE(std::filesystem::exists(at.resolved));
        REQUIRE_FALSE(read.failure.has_value());
        CHECK(read.values.text(optimization::source_key).value_or(std::string()) == "manual");
        CHECK(read.values.text(optimization::expression_key).value_or(std::string()) == expression);
        CHECK(read.values.identities(optimization::parameters_key) == std::vector<std::string>{letter});
        for(const char *leaf : {optimization::parameter_value_leaf, optimization::parameter_lower_leaf, optimization::parameter_upper_leaf})
            CHECK(read.values.origin_of(read.values.key(optimization::parameters_key, letter, leaf).value_or(std::string())).kind == config::origin_kind::source);
    }
}

TEST_CASE("each shipped typed document publishes a converged run from its seed through the demonstration's registration", "[examples][optimization]")
{
    const scratch where("shipped-optimization-typed");
    const offered shipped(where);

    for(const std::string named : {"Typed: 2x^3 + 5b", "Typed: x^2 + ay^2 - 1"})
    {
        INFO(named);
        REQUIRE(std::ranges::find(shipped.names, named) != shipped.names.end());

        const std::string drawn = solve_text(*shipped.registry, named);
        CHECK(drawn.find("Converged. Steps taken: ") != std::string::npos);
        CHECK(drawn.find("Converged. Steps taken: 0.") == std::string::npos);
    }
}
