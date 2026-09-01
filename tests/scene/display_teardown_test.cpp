#include "praxis/scene/preset.h"
#include "praxis/scene/stencil.h"
#include "praxis/scene/log_buffer.h"
#include "praxis/scene/visualizer.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <string>
#include <vector>
#include <functional>

using namespace praxis::scene;
using namespace praxis::scheduler;

namespace {

struct body_record
{
    int torn_down;
    int rendered;
};

class recording_body : public stencil
{
public:
    explicit recording_body(body_record &into)
            : m_into(into)
    {
    }

    praxis::expected<void, praxis::refusal> initialize() override
    {
        return {};
    }

    void tear_down() override
    {
        ++m_into.torn_down;
    }

    void render() const override
    {
        ++m_into.rendered;
    }

private:
    body_record &m_into;
};

// The composition keeps the route its site offered it, so a case asks for the unload through a copy
// of that route rather than through a share of what the composition owns.
preset_registry::factory composing(body_record &into, std::function<void()> &kept)
{
    return [&into, &kept](const preset_site &site)
    {
        kept = site.ask_unload;
        return std::make_shared<preset>(std::make_shared<recording_body>(into), std::vector<std::shared_ptr<imgui_window>>{}, site.add_window, site.remove_window);
    };
}

// One live stage: a scheduler with no worker of its own, over a real renderer, a real canvas and a
// real window list. The drain services every posted load, unload and frame, so a case runs on the
// calling thread alone with no sleep and no wall clock in it. The record is declared ahead of the
// renderer so that it outlives the body reporting into it.
struct stage
{
    stage()
            : loop(inline_workers)
            , body{}
            , asked()
            , view(std::make_shared<preset_registry>(), loop)
    {
    }

    void load(const preset_registry::factory &builder)
    {
        view.load_preset(builder);
        REQUIRE(loop.drain().has_value());
    }

    void unload()
    {
        asked();
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
    body_record body;
    std::function<void()> asked;
    visualizer view;
};

preset_registry::factory answering_nothing()
{
    return [](const preset_site &) { return std::shared_ptr<preset>(); };
}

}

TEST_CASE("an unload asked for through a composition's own route needs no frame to take effect", "[scene][display]")
{
    stage live;

    live.load(composing(live.body, live.asked));
    REQUIRE(live.view.is_preset_loaded());

    live.frame();
    live.frame();
    REQUIRE(live.body.rendered == 2);

    live.unload();

    REQUIRE_FALSE(live.view.is_preset_loaded());
    REQUIRE(live.body.torn_down == 1);
    REQUIRE(live.body.rendered == 2);
}

TEST_CASE("a body torn down is no longer rendered by the frames that follow", "[scene][display]")
{
    stage live;

    live.load(composing(live.body, live.asked));
    live.frame();
    live.frame();
    live.unload();

    live.frame();
    live.frame();

    REQUIRE(live.body.rendered == 2);
    REQUIRE(live.body.torn_down == 1);
}

TEST_CASE("a composition refused while one is held leaves the held one loaded and rendering", "[scene][display]")
{
    stage live;

    live.load(composing(live.body, live.asked));
    live.frame();
    live.load(answering_nothing());
    live.frame();

    REQUIRE(live.view.is_preset_loaded());
    REQUIRE(live.body.torn_down == 0);
    REQUIRE(live.body.rendered == 2);
}

// Classifying a refusal as fatal belongs to whichever extension owns the operation refused, and this
// target links none of them. What is joined here is the pair of surfaces on this side of that
// boundary: the report the library writes reaches the ring a message window drains, and the route the
// composition was given unloads it.
TEST_CASE("a reported refusal reaches the message ring and the route unloads the composition", "[scene][display]")
{
    stage live;

    // The renderer installs a ring of its own while it is built, so this one is added behind it.
    const std::shared_ptr<log_buffer> messages = std::make_shared<log_buffer>(default_log_capacity);
    REQUIRE(install_log_sink(messages) > 1u);

    live.load(composing(live.body, live.asked));
    live.frame();
    messages->drain();
    live.load(answering_nothing());

    bool reported = false;
    for(const log_entry &entry : messages->drain())
        if(entry.text.find("its composition was refused") != std::string::npos)
            reported = true;

    live.unload();

    REQUIRE(reported);
    REQUIRE_FALSE(live.view.is_preset_loaded());
    REQUIRE(live.body.torn_down == 1);
}

TEST_CASE("a composition loaded after an unload runs frames of its own", "[scene][display]")
{
    body_record next{};
    std::function<void()> again;
    stage live;

    live.load(composing(live.body, live.asked));
    live.frame();
    live.unload();
    REQUIRE_FALSE(live.view.is_preset_loaded());

    live.load(composing(next, again));
    live.frame();
    live.frame();

    REQUIRE(live.view.is_preset_loaded());
    REQUIRE(next.rendered == 2);
    REQUIRE(next.torn_down == 0);
    REQUIRE(live.body.rendered == 1);
    REQUIRE(live.body.torn_down == 1);
}
