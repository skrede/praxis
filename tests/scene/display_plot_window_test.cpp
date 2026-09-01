#include "praxis/scene/preset.h"
#include "praxis/scene/stencil.h"
#include "praxis/scene/visualizer.h"
#include "praxis/scene/plot_window.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include "praxis/scheduler/strand.h"
#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <memory>
#include <string>
#include <vector>
#include <utility>

using namespace praxis::scene;
using namespace praxis::scheduler;

namespace {

using curves  = std::vector<plot_series>;
using stacked = std::vector<plot_frame>;

const char *const linear_title      = "Time scaling";
const char *const logarithmic_title = "Convergence";
const char *const stacked_title     = "Derivatives";

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

plot_source answering(plot_reading given)
{
    return [given = std::move(given)] { return given; };
}

plot_series curve_named(const std::string &name, double first)
{
    return plot_series{name, {0.0, 1.0, 2.0, 3.0}, {first, first + 2.0, first + 4.0, first + 8.0}};
}

plot_reading two_curves()
{
    return plot_reading{"", "s", stacked{plot_frame{"value", false, curves{curve_named("cubic", 1.0), curve_named("quintic", 16.0)}}}};
}

plot_reading three_frames()
{
    return plot_reading{"", "s",
                        stacked{plot_frame{"path", false, curves{curve_named("value", 1.0)}}, plot_frame{"rate", false, curves{curve_named("value", 16.0)}},
                                plot_frame{"change of rate", false, curves{curve_named("value", 32.0)}}}};
}

// Four decades of ordinate, which is the span a logarithmic scale exists to show.
plot_reading four_decades()
{
    const plot_series falling{"residual", {0.0, 1.0, 2.0, 3.0, 4.0}, {1.0, 0.1, 0.01, 0.001, 0.0001}};

    return plot_reading{"", "iteration", stacked{plot_frame{"error", true, curves{falling}}}};
}

preset_registry::factory plotting()
{
    return [](const preset_site &site)
    {
        std::vector<std::shared_ptr<imgui_window>> panels;
        panels.push_back(std::make_shared<plot_window>(linear_title, nullptr, answering(two_curves())));
        panels.push_back(std::make_shared<plot_window>(logarithmic_title, nullptr, answering(four_decades())));
        panels.push_back(std::make_shared<plot_window>(stacked_title, nullptr, answering(three_frames())));

        return std::make_shared<preset>(std::make_shared<inert_body>(), std::move(panels), site.add_window, site.remove_window);
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

// The window list is private, so what a drawn frame put on screen is read back through the settings
// the GUI library keeps for every window it created this session.
bool drawn_panel(const std::string &settings, const std::string &title)
{
    return settings.find("[Window][" + title + "]") != std::string::npos;
}

}

TEST_CASE("a composition carrying three plot windows draws each of them on a real display", "[scene][display]")
{
    stage live;

    live.load(plotting());
    live.frame();
    live.frame();

    const std::string settings(ImGui::SaveIniSettingsToMemory(nullptr));

    REQUIRE(live.view.is_preset_loaded());
    REQUIRE(drawn_panel(settings, linear_title));
    REQUIRE(drawn_panel(settings, logarithmic_title));
    REQUIRE(drawn_panel(settings, stacked_title));
}

TEST_CASE("the frame keeps being drawn once the composition carrying the plots is released", "[scene][display]")
{
    stage live;

    live.load(plotting());
    live.frame();
    live.frame();
    REQUIRE(live.view.is_preset_loaded());

    live.view.clear_preset();
    REQUIRE(live.loop.drain().has_value());

    live.frame();

    REQUIRE_FALSE(live.view.is_preset_loaded());
}
