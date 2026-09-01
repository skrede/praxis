#include "praxis/scene/preset.h"
#include "praxis/scene/stencil.h"
#include "praxis/scene/log_buffer.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <stdexcept>

namespace {

class silent_body : public praxis::scene::stencil
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

class silent_window : public praxis::scene::imgui_window
{
public:
    explicit silent_window(std::string name)
            : imgui_window(std::move(name))
    {
    }

    void render() override
    {
    }
};

using window_share = std::shared_ptr<praxis::scene::imgui_window>;

praxis::scene::window_route wired()
{
    return [](const window_share &) {};
}

}

TEST_CASE("a composition given no body to render is refused where it is wired", "[scene]")
{
    REQUIRE_THROWS_AS(praxis::scene::preset(nullptr, std::vector<window_share>{}, wired(), wired()), std::invalid_argument);
}

TEST_CASE("a composition given a window that is not there is refused where it is wired", "[scene]")
{
    const std::vector<window_share> panels{std::make_shared<silent_window>("Present"), nullptr};

    REQUIRE_THROWS_AS(praxis::scene::preset(std::make_shared<silent_body>(), panels, wired(), wired()), std::invalid_argument);
}

TEST_CASE("a composition given no route to register its windows through is refused where it is wired", "[scene]")
{
    const std::vector<window_share> panels{std::make_shared<silent_window>("Present")};

    REQUIRE_THROWS_AS(praxis::scene::preset(std::make_shared<silent_body>(), panels, nullptr, wired()), std::invalid_argument);
    REQUIRE_THROWS_AS(praxis::scene::preset(std::make_shared<silent_body>(), panels, wired(), nullptr), std::invalid_argument);
}

TEST_CASE("a composition given a body and every window it names is constructed", "[scene]")
{
    const std::vector<window_share> panels{std::make_shared<silent_window>("Present")};
    const praxis::scene::preset composed(std::make_shared<silent_body>(), panels, wired(), wired());

    REQUIRE(composed.stencil != nullptr);
    REQUIRE(composed.windows.size() == 1);
}

TEST_CASE("the window sink given no buffer to write into is refused at installation", "[scene]")
{
    REQUIRE_THROWS_AS(praxis::scene::install_log_sink(nullptr), std::invalid_argument);
}
