#include "scratch_directory.h"

#include "imgui_frame.h"
#include "panel_keys.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <threepp/scenes/Scene.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <fstream>
#include <utility>
#include <filesystem>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr const char *opening_document = "<optimization><preset name=\"saved\" scenario=\"curve\"/><function name=\"x^3 - 2x - 5\"/><range x_lower=\"-4\" x_upper=\"4\"/>"
                                         "<seed x=\"3\"/><solve limit=\"70\" step=\"0\"/></optimization>";

std::vector<optimization::registered_curve> two_curves()
{
    return {optimization::registered_curve(
                    "x^3 - 2x - 5", [](double x) { return x * x * x - 2.0 * x - 5.0; }, [](double x) { return 3.0 * x * x - 2.0; }),
            optimization::registered_curve("cos(x) - x", [](double x) { return std::cos(x) - x; }, [](double x) { return -std::sin(x) - 1.0; })};
}

config::location written(const std::string &file)
{
    const std::filesystem::path directory = shared_scratch_directory() / "optimization_saved";
    std::filesystem::create_directories(directory);
    std::ofstream(directory / file, std::ios::binary | std::ios::trunc) << opening_document << "\n";

    return config::resolve(file, directory);
}

config::document read(const config::location &at)
{
    const config::outcome carried = config::load_or_defaults(config::binding{presets::optimization_keyspace(), at, config::expectation::partial});
    REQUIRE_FALSE(carried.failure.has_value());

    return carried.values;
}

// The scenario a document opens through the registration, with its windows drawn one at a time.
struct composed_scenario
{
    scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    scene::preset_site site;
    std::shared_ptr<scene::preset> preset;

    explicit composed_scenario(const config::location &at)
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
        REQUIRE(presets::register_optimization(registry, std::vector<config::location>{at}, nullptr, nullptr, optimization::baseline(), two_curves(), false).size() == 1u);

        preset = registry->load_preset("saved")(site);
        REQUIRE(preset != nullptr);
        REQUIRE(preset->initialize().has_value());
        REQUIRE(loop.drain().has_value());
    }

    composed_scenario(const composed_scenario &)            = delete;
    composed_scenario &operator=(const composed_scenario &) = delete;

    ~composed_scenario()
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

    std::vector<config::edit> offered(const config::document &carried) const
    {
        std::vector<const config::configurable *> shown;
        for(const std::shared_ptr<scene::imgui_window> &window : preset->windows)
            if(const config::configurable *one = window->as_configurable(); one != nullptr)
                shown.push_back(one);

        return config::shown_edits(shown, carried);
    }
};

void edit_function_and_range(tests::imgui_frame &frames, const drawing &function)
{
    REQUIRE(entry_taken_at(frames, function, 1u));
    type_into(frames, function, 2u, "-3");
    type_into(frames, function, 3u, "3.5");
}

// Run is reached by a step sideways, since the step below the limit field may land on Step once.
void edit_and_run(tests::imgui_frame &frames, const drawing &solve)
{
    type_into(frames, solve, 0u, "0.5");
    type_into(frames, solve, 1u, "40");
    stand_below_top(frames, solve, 2u);
    tap(frames, solve, ImGuiKey_LeftArrow);
    tap(frames, solve, ImGuiKey_Space);
}

template<typename value>
value read_back(const config::document &values, const char *key, expected<value, config::error> (config::document::*reading)(std::string_view) const)
{
    return (values.*reading)(key).value_or(value());
}

void check_edited(const config::document &reopened)
{
    CHECK(read_back<std::string>(reopened, "function/name", &config::document::text) == "cos(x) - x");
    CHECK(read_back<double>(reopened, "range/x_lower", &config::document::real) == -3.0);
    CHECK(read_back<double>(reopened, "range/x_upper", &config::document::real) == 3.5);
    CHECK(read_back<double>(reopened, "seed/x", &config::document::real) == 0.5);
    CHECK(read_back<std::int64_t>(reopened, "solve/limit", &config::document::integer) == 40);
    CHECK(read_back<std::int64_t>(reopened, "solve/step", &config::document::integer) == 2);
}

}

TEST_CASE("the function, its range, the seed, the limit and the selected step are saved and reopened", "[presets][optimization][documents]")
{
    const config::location at = written("saved.xml");
    {
        composed_scenario first(at);
        tests::imgui_frame frames;
        edit_function_and_range(frames, first.draw("Function"));
        edit_and_run(frames, first.draw("Solve"));
        REQUIRE(first.loop.drain().has_value());
        tweak_from(frames, first.draw("Solve"), 3u, 2u);
        REQUIRE(first.loop.drain().has_value());

        REQUIRE(config::save(presets::optimization_keyspace(), at, first.offered(read(at))).has_value());
    }

    const config::document reopened = read(at);
    check_edited(reopened);

    const composed_scenario second(at);
    CHECK(second.offered(reopened).empty());
}

TEST_CASE("a scenario opened and left as it opened offers nothing to save", "[presets][optimization][documents]")
{
    const config::location at = written("untouched.xml");
    const composed_scenario opened(at);

    CHECK(opened.offered(read(at)).empty());
}
