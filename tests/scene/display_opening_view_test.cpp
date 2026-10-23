#include "praxis/scene/preset.h"
#include "praxis/scene/stencil.h"
#include "praxis/scene/visualizer.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/math/Vector3.hpp>

#include <cmath>
#include <memory>
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

preset_registry::factory opening_at(std::optional<camera_view> view)
{
    return [view](const preset_site &site)
    {
        auto built          = std::make_shared<preset>(std::make_shared<inert_body>(), std::vector<std::shared_ptr<imgui_window>>{}, site.add_window, site.remove_window);
        built->opening_view = view;

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
        REQUIRE(view.is_preset_loaded());
    }

    scheduler loop;
    visualizer view;
};

void check_at(const threepp::Vector3 &seen, const threepp::Vector3 &expected)
{
    CHECK(std::abs(seen.x - expected.x) <= 1.0e-5f);
    CHECK(std::abs(seen.y - expected.y) <= 1.0e-5f);
    CHECK(std::abs(seen.z - expected.z) <= 1.0e-5f);
}

const camera_view named{threepp::Vector3(1.f, 2.f, 3.f), threepp::Vector3(0.f, 0.5f, 0.f)};

}

TEST_CASE("a visualizer that has loaded nothing stands its camera over the origin and turns about it", "[scene][visualizer][display]")
{
    stage staged;
    const camera_view seen = staged.view.current_view();
    check_at(seen.position, threepp::Vector3(0.f, 1.25f, 1.f));
    check_at(seen.target, threepp::Vector3(0.f, 0.f, 0.f));
}

TEST_CASE("a preset naming an opening view moves the camera and the point it turns about there on load", "[scene][visualizer][display]")
{
    stage staged;
    staged.load(opening_at(named));
    const camera_view seen = staged.view.current_view();
    check_at(seen.position, named.position);
    check_at(seen.target, named.target);
}

TEST_CASE("a preset naming no opening view leaves the camera where the previous one put it", "[scene][visualizer][display]")
{
    stage staged;
    staged.load(opening_at(named));
    staged.load(opening_at(std::nullopt));
    const camera_view seen = staged.view.current_view();
    check_at(seen.position, named.position);
    check_at(seen.target, named.target);
}
