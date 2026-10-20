#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"
#include "scratch_directory.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"

#include "praxis/scheduler/strand.h"
#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/objects/Mesh.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <utility>
#include <algorithm>
#include <filesystem>

using namespace praxis;
using namespace praxis::fixture;

namespace {

// Every point the registered surface's gradient was asked at, in order: the solve asks it at each
// step's start and the publication at every iterate, so the first is the run's seed.
struct evaluations
{
    std::vector<Eigen::Vector2d> valued;
    std::vector<Eigen::Vector2d> sloped;
};

optimization::registered_surface recording(const std::shared_ptr<evaluations> &into)
{
    return optimization::registered_surface(
            "x^2 + y^2 - 1",
            [into](double x, double y)
            {
                into->valued.emplace_back(x, y);
                return x * x + y * y - 1.0;
            },
            [into](double x, double y)
            {
                into->sloped.emplace_back(x, y);
                return Eigen::Vector2d(2.0 * x, 2.0 * y);
            });
}

config::location written(const std::string &file, const std::string &body)
{
    const std::filesystem::path directory = shared_scratch_directory() / "optimization-surface";
    std::filesystem::create_directories(directory);
    std::ofstream(directory / file, std::ios::binary | std::ios::trunc) << body << "\n";

    return config::resolve(file, directory);
}

constexpr const char *surface_document = "<optimization><preset name=\"bowl\" scenario=\"surface\"/><function name=\"x^2 + y^2 - 1\"/>"
                                         "<range x_lower=\"-2\" x_upper=\"2\" y_lower=\"-2\" y_upper=\"2\"/><seed x=\"-2\" y=\"-1.5\"/></optimization>";

// One composition of the registered surface's document, initialized and run, on a scene of its own.
struct composed_surface
{
    composed_surface()
            : seen(std::make_shared<evaluations>())
            , loop(scheduler::inline_workers)
            , scene(threepp::Scene::create())
    {
        const auto registry = std::make_shared<scene::preset_registry>();
        const std::vector<optimization::registered_surface> surfaces{recording(seen)};
        named = presets::register_optimization(registry, std::vector<config::location>{written("bowl.xml", surface_document)}, nullptr, nullptr, optimization::baseline(), {}, surfaces,
                                               false);
        REQUIRE(named == std::vector<std::string>{"bowl"});

        const scene::window_route nowhere = [](const std::shared_ptr<scene::imgui_window> &) {};
        built                             = registry->load_preset("bowl")(scene::preset_site{*scene, loop.main_strand(), *loop.make_strand(), [](std::string) {}, nowhere, nowhere, {}});
        REQUIRE(built != nullptr);
        REQUIRE(built->initialize().has_value());
        REQUIRE(loop.drain().has_value());
        built->stencil->render();
    }

    ~composed_surface()
    {
        built->tear_down();
        static_cast<void>(loop.retire_strand(built->work, std::move(built->release_cb)));
        static_cast<void>(loop.drain());
    }

    drawing draw(const std::string &window) const
    {
        const auto found = std::ranges::find_if(built->windows, [&window](const std::shared_ptr<scene::imgui_window> &held) { return held->display_name() == window; });
        REQUIRE(found != built->windows.end());

        return [shown = *found] { shown->render(); };
    }

    threepp::Object3D &drawn(const char *name) const
    {
        threepp::Object3D *found = scene->getObjectByName(name);
        REQUIRE(found != nullptr);

        return *found;
    }

    std::shared_ptr<evaluations> seen;
    scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::vector<std::string> named;
    std::shared_ptr<scene::preset> built;
};

}

TEST_CASE("a surface document opens the surface scenario over the registered surfaces and draws its run", "[presets][optimization][surface]")
{
    composed_surface opened;

    std::vector<std::string> windows;
    for(const std::shared_ptr<scene::imgui_window> &window : opened.built->windows)
        windows.push_back(window->display_name());
    CHECK(windows == std::vector<std::string>{"Function", "Graph", "Solve", "Linearization"});

    REQUIRE_FALSE(opened.seen->sloped.empty());
    CHECK(opened.seen->sloped.front() == Eigen::Vector2d(-2.0, -1.5));
    CHECK(std::ranges::find(opened.seen->valued, Eigen::Vector2d(2.0, 2.0)) != opened.seen->valued.end());

    const auto *field = dynamic_cast<threepp::Mesh *>(&opened.drawn("surface"));
    REQUIRE(field != nullptr);
    CHECK(field->geometry()->drawRange.count > 0);
    CHECK(field->visible);
    CHECK(opened.drawn("iterates").visible);
}

TEST_CASE("the function window over a surface takes both ranges and the solve window both seed coordinates", "[presets][optimization][surface]")
{
    composed_surface opened;
    tests::imgui_frame frames;

    type_into(frames, opened.draw("Function"), 5, "3");
    REQUIRE(opened.loop.drain().has_value());
    CHECK(std::ranges::find(opened.seen->valued, Eigen::Vector2d(2.0, 3.0)) != opened.seen->valued.end());

    type_into(frames, opened.draw("Solve"), 1, "1.5");
    press_on(frames, opened.draw("Solve"), "Solve", "Run");
    opened.seen->sloped.clear();
    REQUIRE(opened.loop.drain().has_value());
    REQUIRE_FALSE(opened.seen->sloped.empty());
    CHECK(opened.seen->sloped.front() == Eigen::Vector2d(-2.0, 1.5));
}

TEST_CASE("the graph window's switches hide and show the surface and its iterates", "[presets][optimization][surface]")
{
    composed_surface opened;
    tests::imgui_frame frames;

    press_on(frames, opened.draw("Graph"), "Graph", "Surface");
    opened.built->stencil->render();
    CHECK_FALSE(opened.drawn("surface").visible);
    CHECK(opened.drawn("iterates").visible);

    press_on(frames, opened.draw("Graph"), "Graph", "Iterates");
    opened.built->stencil->render();
    CHECK_FALSE(opened.drawn("iterates").visible);

    press_on(frames, opened.draw("Graph"), "Graph", "Surface");
    opened.built->stencil->render();
    CHECK(opened.drawn("surface").visible);
}
