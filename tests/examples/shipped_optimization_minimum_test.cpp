#include "scratch_documents.h"
#include "offered_optimization.h"

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

#include <threepp/scenes/Scene.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>

using namespace praxis;
using namespace scratch_documents;
using namespace offered_optimization;

namespace {

// The text the Solve window draws once the composition's work strand has run.
std::string solve_text(scene::preset_registry &registry, const std::string &named)
{
    std::string drawn;
    on_own_scene(registry, named,
                 [&drawn](scene::preset &composed, scheduler::scheduler &loop, threepp::Scene &)
                 {
                     REQUIRE(loop.drain().has_value());
                     const auto solve = std::ranges::find_if(composed.windows, [](const std::shared_ptr<scene::imgui_window> &window) { return window->display_name() == "Solve"; });
                     REQUIRE(solve != composed.windows.end());
                     tests::imgui_frame frames;
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

TEST_CASE("the shipped minimum-form documents state the minimum form and Newton's method", "[examples][optimization]")
{
    for(const char *document : {"curve-minimum.xml", "surface-minimum.xml"})
    {
        INFO(document);
        const config::location at  = config::resolve(document, PRAXIS_SHIPPED_OPTIMIZATION_DIR);
        const config::outcome read = config::load_or_defaults(presets::optimization_keyspace(), at, config::expectation::partial);
        REQUIRE(std::filesystem::exists(at.resolved));
        REQUIRE_FALSE(read.failure.has_value());
        CHECK(read.values.text(optimization::form_key).value_or(std::string()) == "minimum");
        CHECK(read.values.text(optimization::method_key).value_or(std::string()) == "newton");
        CHECK(read.values.origin_of(optimization::step_length_key).kind == config::origin_kind::source);
    }
}

TEST_CASE("each shipped minimum-form document publishes a converged run from its seed through the demonstration's registration", "[examples][optimization]")
{
    const scratch where("shipped-optimization-minimum");
    const offered shipped(where);

    for(const std::string named : {"Minimization: x^4 - 3x^2 + x", "Minimization: (x^2 + 10y^2)/2"})
    {
        INFO(named);
        REQUIRE(std::ranges::find(shipped.names, named) != shipped.names.end());

        const std::string drawn = solve_text(*shipped.registry, named);
        CHECK(drawn.find("Converged. Steps taken: ") != std::string::npos);
        CHECK(drawn.find("Converged. Steps taken: 0.") == std::string::npos);
    }
}
