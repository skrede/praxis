#include "praxis/scene/preset.h"
#include "praxis/scene/stencil.h"
#include "praxis/scene/visualizer.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include "praxis/scheduler/task.h"
#include "praxis/scheduler/strand.h"
#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <memory>
#include <string>
#include <vector>
#include <optional>

using namespace praxis::scene;
using namespace praxis::scheduler;

namespace {

class inert_body : public stencil
{
public:
    praxis::expected<void, praxis::refusal> initialize() override
    {
        return {};
    }

    void tear_down() override
    {
    }

    void render() const override
    {
    }
};

// A period far enough out that its deadline has not passed by the time the drain asks, so the drain
// reports the pool idle rather than servicing the same task forever.
constexpr step_period unhurried{seconds{1.0}};

// Two admissions of different shapes: one that keeps running and one that is over after its first
// service, which is the pair a panel has to keep telling apart.
preset_registry::factory admitting_work()
{
    return [](const preset_site &site)
    {
        std::shared_ptr<preset> built = std::make_shared<preset>(std::make_shared<inert_body>(), std::vector<std::shared_ptr<imgui_window>>{}, site.add_window, site.remove_window);

        const strand work = site.work;
        built->admit_cb   = [work]() -> praxis::expected<std::vector<task_handle>, praxis::refusal>
        {
            std::vector<task_handle> admitted;
            admitted.push_back(work.every(unhurried, overrun::drop, [](step_delta) {}));
            admitted.push_back(work.after(step_period{seconds{0.0}}, [] {}));

            return admitted;
        };

        return built;
    };
}

struct stage
{
    stage()
            : loop(inline_workers)
            , view(std::make_shared<preset_registry>(), loop)
    {
    }

    void load(const preset_registry::factory &builder)
    {
        view.load_preset(builder);
        REQUIRE(loop.drain().has_value());
    }

    void frame()
    {
        bool drawn = false;
        REQUIRE(view.executor().post([this, &drawn] { drawn = view.render_once(); }).has_value());
        REQUIRE(loop.drain().has_value());
        REQUIRE(drawn);
    }

    scheduler loop;
    visualizer view;
};

bool drawn_panel(const std::string &settings, const std::string &title)
{
    return settings.find("[Window][" + title + "]") != std::string::npos;
}

}

TEST_CASE("the visualizer draws a frame on the strand it reports as its executor", "[scene][display]")
{
    scheduler loop(inline_workers);
    visualizer view(std::make_shared<preset_registry>(), loop);

    bool drawn = false;
    REQUIRE(view.executor().post([&view, &drawn] { drawn = view.render_once(); }).has_value());
    REQUIRE(loop.drain().has_value());

    REQUIRE(drawn);
    REQUIRE_FALSE(view.is_preset_loaded());
}

TEST_CASE("nothing held reports no stepped work at all", "[scene][display]")
{
    stage live;

    REQUIRE_FALSE(live.view.is_preset_loaded());
    REQUIRE(live.view.composed_work().empty());

    live.frame();

    REQUIRE(live.view.composed_work().empty());
}

TEST_CASE("what a loaded composition admitted is reported in the order it holds it", "[scene][display]")
{
    stage live;

    live.load(admitting_work());

    const std::vector<stepped_work_report> reported = live.view.composed_work();
    REQUIRE(reported.size() == 2u);
    REQUIRE(reported[0].valid);
    REQUIRE(reported[0].active);
    REQUIRE(reported[0].counters.policy.has_value());
    REQUIRE(*reported[0].counters.policy == overrun::drop);
}

// The receipt is held by the preset for as long as the preset lives, so a one-shot that has run is
// still reported and says so through its state rather than by leaving the report.
TEST_CASE("a one-shot that has completed is still reported, inactive", "[scene][display]")
{
    stage live;

    live.load(admitting_work());
    REQUIRE(live.loop.drain().has_value());

    const std::vector<stepped_work_report> reported = live.view.composed_work();
    REQUIRE(reported.size() == 2u);
    REQUIRE(reported[1].valid);
    REQUIRE_FALSE(reported[1].active);
}

// The window list is private, so what a drawn frame put on screen is read back through the settings
// the GUI library keeps for every window it created this session. A window asking for none of those
// settings is invisible here, which is why the view gizmo is not among the two named.
TEST_CASE("a drawn frame puts both the selector and the work panel on screen", "[scene][display]")
{
    stage live;

    live.frame();
    const std::string settings(ImGui::SaveIniSettingsToMemory(nullptr));

    REQUIRE(drawn_panel(settings, "Presets"));
    REQUIRE(drawn_panel(settings, "Stepped work"));
}

// The orbit controls are held privately and the only window built over them keeps no settings, so
// what a suite can reach is the construction path: the gizmo takes them by reference while the
// visualizer is built, and every frame below draws it.
TEST_CASE("frames keep being drawn over the controls the visualizer was constructed with", "[scene][display]")
{
    stage live;

    live.frame();
    live.frame();
    live.load(admitting_work());
    live.frame();

    REQUIRE(live.view.is_preset_loaded());
}

TEST_CASE("a visualizer opens at the size it is given and answers what it was left at", "[scene][display]")
{
    constexpr int asked_width  = 720;
    constexpr int asked_height = 540;

    scheduler loop(inline_workers);
    visualizer view(std::make_shared<preset_registry>(), loop,
                    {.view = visualizer::projection::perspective, .messages = nullptr, .window = {.width = asked_width, .height = asked_height, .x = 120, .y = 80}, .root = {}});

    bool drawn = false;
    REQUIRE(view.executor().post([&view, &drawn] { drawn = view.render_once(); }).has_value());
    REQUIRE(loop.drain().has_value());
    REQUIRE(drawn);

    const visualizer::geometry where = view.window_geometry();
    REQUIRE(where.width == asked_width);
    REQUIRE(where.height == asked_height);

    // A window manager places a window where it chooses, so what is portable is that a position is
    // reported at all rather than that it is the one asked for.
    CHECK(where.x.has_value());
    CHECK(where.y.has_value());
}
