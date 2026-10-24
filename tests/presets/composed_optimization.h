#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_COMPOSED_OPTIMIZATION_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_COMPOSED_OPTIMIZATION_H

#include "panel_keys.h"
#include "imgui_frame.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace praxis::fixture {

inline void check_parameter(const optimization::typed_parameter &read, char name, double value, double lower, double upper)
{
    CHECK(read.name == name);
    CHECK(read.value == value);
    CHECK(read.range.lower == lower);
    CHECK(read.range.upper == upper);
}

inline config::document read_optimization(const config::location &at)
{
    const config::outcome carried = config::load_or_defaults(config::binding{presets::optimization_keyspace(), at, config::expectation::partial});
    REQUIRE_FALSE(carried.failure.has_value());

    return carried.values;
}

// The scenario a document opens through the registration as the preset it names, with its windows
// drawn one at a time.
struct composed_optimization
{
    scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    scene::preset_site site;
    std::shared_ptr<scene::preset> preset;

    composed_optimization(const config::location &at, std::vector<optimization::registered_curve> curves, const std::string &named,
                          std::vector<optimization::registered_surface> surfaces = {})
            : loop(scheduler::inline_workers)
            , scene(threepp::Scene::create())
            , site{*scene,
                   loop.main_strand(),
                   *loop.make_strand(),
                   [](std::string) {},
                   [](const std::shared_ptr<scene::imgui_window> &) {},
                   [](const std::shared_ptr<scene::imgui_window> &) {},
                   {}}
            , preset()
    {
        const auto registry = std::make_shared<scene::preset_registry>();
        REQUIRE(presets::register_optimization(registry, std::vector<config::location>{at}, nullptr, nullptr, optimization::baseline(), curves, surfaces, false).size() == 1u);

        preset = registry->load_preset(named)(site);
        REQUIRE(preset != nullptr);
        REQUIRE(preset->initialize().has_value());
        REQUIRE(loop.drain().has_value());
    }

    composed_optimization(const composed_optimization &)            = delete;
    composed_optimization &operator=(const composed_optimization &) = delete;

    ~composed_optimization()
    {
        preset->tear_down();
        static_cast<void>(loop.retire_strand(preset->work, std::move(preset->release_cb)));
        static_cast<void>(loop.drain());
    }

    drawing draw(const std::string &named) const
    {
        for(const std::shared_ptr<scene::imgui_window> &window : preset->windows)
            if(window->display_name() == named)
                return [window] { window->render(); };
        FAIL("no window is named " << named);

        return {};
    }

    // What the window writes as text when drawn once more.
    std::string written(tests::imgui_frame &frames, const std::string &named) const
    {
        std::string drawn;
        const drawing shown = draw(named);
        frames.draw_frame(
                [&shown, &drawn]
                {
                    ImGui::LogToBuffer();
                    shown();
                    drawn = GImGui->LogBuffer.c_str();
                    ImGui::LogFinish();
                });

        return drawn;
    }

    // The edits the window itself offers, before the store drops any it cannot carry.
    std::vector<config::edit> edits_of(const std::string &named, const config::document &carried) const
    {
        for(const std::shared_ptr<scene::imgui_window> &window : preset->windows)
            if(window->display_name() == named && window->as_configurable() != nullptr)
                return window->as_configurable()->settings_edits(carried);
        FAIL("no window offering edits is named " << named);

        return {};
    }

    std::vector<config::edit> offered(const config::document &carried) const
    {
        std::vector<const config::configurable *> shown;
        for(const std::shared_ptr<scene::imgui_window> &window : preset->windows)
            if(const config::configurable *one = window->as_configurable(); one != nullptr)
                shown.push_back(one);

        return config::shown_edits(shown, carried);
    }
};

}

#endif
