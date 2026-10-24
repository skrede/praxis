#include "../presets/scratch_directory.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/stencil.h"
#include "praxis/scene/visualizer.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <memory>
#include <string>
#include <vector>
#include <filesystem>

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

const std::string held_name = "A preset whose name runs well past the selector's default width";

std::shared_ptr<preset_registry> one_preset()
{
    auto registry = std::make_shared<preset_registry>();
    registry->register_preset(held_name, [](const preset_site &site)
                              { return std::make_shared<preset>(std::make_shared<inert_body>(), std::vector<std::shared_ptr<imgui_window>>{}, site.add_window, site.remove_window); });

    return registry;
}

visualizer::options rooted_at(const std::filesystem::path &root)
{
    visualizer::options chosen{};
    chosen.root = root;

    return chosen;
}

void draw(scheduler &loop, visualizer &view)
{
    bool drawn = false;
    REQUIRE(view.executor().post([&view, &drawn] { drawn = view.render_once(); }).has_value());
    REQUIRE(loop.drain().has_value());
    REQUIRE(drawn);
}

}

TEST_CASE("the selector opens with no saved layout wide enough for its whole row", "[scene][visualizer][display]")
{
    const std::filesystem::path root = praxis::fixture::scratch_directory();
    {
        scheduler loop(inline_workers);
        visualizer view(one_preset(), loop, rooted_at(root));
        view.saving_through([] {});
        view.load_preset(held_name);
        REQUIRE(loop.drain().has_value());
        REQUIRE(view.is_preset_loaded());
        REQUIRE(view.save_offered());

        for(int frame = 0; frame < 3; ++frame)
            draw(loop, view);

        const ImGuiWindow *selector = ImGui::FindWindowByName("Presets");
        REQUIRE(selector != nullptr);
        CHECK(selector->DC.CursorMaxPos.x <= selector->InnerClipRect.Max.x);
    }
    std::filesystem::remove_all(root);
}
