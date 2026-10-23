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

#include <array>
#include <tuple>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <numeric>
#include <utility>
#include <algorithm>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace scratch_documents;
using namespace offered_optimization;

namespace {

const std::vector<std::string> curve_windows{"Function##curve", "Graph##curve", "Display##curve", "Solve##curve", "Linearization##curve"};
const std::vector<std::string> surface_windows{"Function##surface", "Display##surface", "Solve##surface", "Linearization##surface"};

// Each shipped preset by the name it registers under, and whether its scenario is a surface.
constexpr std::array<std::pair<std::string_view, bool>, 6> shipped_scenarios{{{"Root finding: a cubic", false},
                                                                              {"Root finding: a paraboloid", true},
                                                                              {"Minimization: x^4 - 3x^2 + x", false},
                                                                              {"Minimization: (x^2 + 10y^2)/2", true},
                                                                              {"Typed: 2x^3 + 5b", false},
                                                                              {"Typed: x^2 + ay^2 - 1", true}}};

// What each window of one composition draws before and after its work strand has run, the windows
// it opened, and the vertices of the surface field it drew.
struct drawn
{
    std::vector<std::string> windows;
    std::vector<int> before;
    std::vector<int> after;
    int field;
};

std::vector<int> vertices_of(const std::vector<std::shared_ptr<scene::imgui_window>> &windows)
{
    std::vector<int> counted;
    for(const std::shared_ptr<scene::imgui_window> &window : windows)
    {
        tests::imgui_frame frames;
        frames.draw([&window] { window->render(); });
        counted.push_back(frames.vertices());
    }

    return counted;
}

drawn composed_and_drawn(scene::preset_registry &registry, const std::string &named)
{
    drawn seen{{}, {}, {}, 0};
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

// A curve's run is drawn in its graph and a surface's in its field; either changes what its windows draw.
void check_drawn(const drawn &seen, bool surface)
{
    REQUIRE(seen.windows == (surface ? surface_windows : curve_windows));
    REQUIRE(seen.after.size() == seen.windows.size());
    CHECK(std::accumulate(seen.after.begin(), seen.after.end(), 0) > std::accumulate(seen.before.begin(), seen.before.end(), 0));
    if(surface)
    {
        CHECK(seen.field > 0);
        return;
    }
    const auto graph = static_cast<std::size_t>(std::ranges::find(curve_windows, "Graph##curve") - curve_windows.begin());
    CHECK(seen.field == 0);
    CHECK(seen.after[graph] > seen.before[graph]);
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

    REQUIRE(shipped.names.size() == shipped_scenarios.size());
    for(const auto &[named, surface] : shipped_scenarios)
    {
        INFO(named);
        REQUIRE(std::ranges::find(shipped.names, named) != shipped.names.end());
        check_drawn(composed_and_drawn(*shipped.registry, std::string(named)), surface);
    }
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
