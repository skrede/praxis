#include "praxis/scene/preset.h"
#include "praxis/scene/stencil.h"
#include "praxis/scene/composition.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/compat/detail/callable.h"
#include "praxis/compat/expected.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/extension/refusal.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>
#include <threepp/core/Object3D.hpp>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <functional>
#include <string_view>

using namespace praxis::scene;
using namespace praxis::scheduler;

namespace {

using window_share = std::shared_ptr<imgui_window>;

time_point dictated{};

time_point reading()
{
    return dictated;
}

clock_source dictating()
{
    dictated = time_point{};
    return clock_source{&reading};
}

// What the composed body did, kept outside the body so a case can read it without holding a share
// of anything the composition owns.
struct body_record
{
    int initialized;
    int torn_down;
    int rendered;
    bool destroyed;
};

// What every body and every window route of one stage did, in the order they did it. A count says
// how often and cannot say which came first, which is the whole of what a switch claims.
std::vector<std::string> sequence;

class recording_body : public stencil
{
public:
    recording_body(body_record &into, threepp::Scene &target, std::string label)
            : m_into(into)
            , m_label(std::move(label))
            , m_target(target)
            , m_node(threepp::Object3D::create())
    {
    }

    ~recording_body() override
    {
        m_into.destroyed = true;
    }

    praxis::expected<void, praxis::refusal> initialize() override
    {
        ++m_into.initialized;
        sequence.push_back(m_label + " initialized");
        m_target.add(m_node);

        return {};
    }

    void tear_down() override
    {
        ++m_into.torn_down;
        sequence.push_back(m_label + " torn down");
        m_target.remove(*m_node);
    }

    void render() const override
    {
        ++m_into.rendered;
    }

private:
    body_record &m_into;
    std::string m_label;
    threepp::Scene &m_target;
    std::shared_ptr<threepp::Object3D> m_node;
};

class silent_window : public imgui_window
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

class settled_window : public imgui_window, public praxis::config::configurable
{
public:
    settled_window(std::string name, std::string path)
            : imgui_window(std::move(name))
            , m_path(std::move(path))
    {
    }

    void render() override
    {
    }

    const praxis::config::configurable *as_configurable() const override
    {
        return this;
    }

    std::string_view settings_path() const override
    {
        return m_path;
    }

    std::vector<praxis::config::edit> settings_edits(const praxis::config::document &) const override
    {
        return {praxis::config::edit{m_path + ".shown", "1"}};
    }

private:
    std::string m_path;
};

std::shared_ptr<preset> compose(body_record &into, const preset_site &site, std::string label = "the body")
{
    return std::make_shared<preset>(std::make_shared<recording_body>(into, site.scene, std::move(label)), std::vector<window_share>{std::make_shared<silent_window>("Panel")},
                                    site.add_window, site.remove_window);
}

// The settings-carrying window sits between two that carry none, so what is collected is neither the
// first window registered nor the last. The case is handed a share of it, which is what lets it tell
// a collection the composition emptied from one whose windows simply died with the preset.
preset_registry::factory settling(body_record &into, std::shared_ptr<settled_window> &kept, std::string path, std::string label = "the body")
{
    return [&into, &kept, path, label](const preset_site &site)
    {
        kept = std::make_shared<settled_window>("Settings", path);

        std::vector<window_share> panels{std::make_shared<silent_window>("Panel"), kept, std::make_shared<silent_window>("Gizmo")};

        return std::make_shared<preset>(std::make_shared<recording_body>(into, site.scene, label), std::move(panels), site.add_window, site.remove_window);
    };
}

// The registration route the site offered, kept so a case can register a window the preset never
// listed and therefore never withdraws.
preset_registry::factory offering(body_record &into, std::shared_ptr<settled_window> &kept, window_route &route, std::string path)
{
    const preset_registry::factory settled = settling(into, kept, std::move(path));

    return [settled, &route](const preset_site &site)
    {
        route = site.add_window;

        return settled(site);
    };
}

// Every composition this builds keeps the route its own site offered it. The case is handed a copy
// of that route rather than a share of anything the composition owns.
preset_registry::factory composing(body_record &into, std::function<void()> &kept, std::string label = "the body")
{
    return [&into, &kept, label](const preset_site &site)
    {
        kept = site.ask_unload;

        return compose(into, site, label);
    };
}

// Where a retirement's acknowledgment ran, which is the strand the retired composition owned and
// never the one the frames are drawn on.
struct acknowledgment
{
    bool ran;
    bool on_its_own;
    bool on_the_render_strand;
};

preset_registry::factory composing_acknowledging(body_record &into, acknowledgment &seen, std::string label)
{
    return [&into, &seen, label](const preset_site &site)
    {
        std::shared_ptr<preset> composed = compose(into, site, label);
        composed->release_cb             = [&seen, own = site.work, render = site.render]
        {
            seen.on_its_own           = own.running_here();
            seen.on_the_render_strand = render.running_here();
            seen.ran                  = true;
        };

        return composed;
    };
}

// A stand-in for whatever a composition puts on the strand it is given. The case observes it
// through a weak pointer, so nothing the case holds keeps it alive.
struct owned_state
{
    int touched;
};

// What the strand's retirement acknowledgment frees, and the flag that freeing sets.
preset_registry::factory composing_owning(body_record &into, bool &released, std::weak_ptr<owned_state> &observed)
{
    return [&into, &released, &observed](const preset_site &site)
    {
        auto owned = std::make_shared<owned_state>();
        observed   = owned;

        std::shared_ptr<preset> composed = compose(into, site);
        composed->release_cb             = [&released, held = std::move(owned)]() mutable
        {
            held.reset();
            released = true;
        };

        return composed;
    };
}

// What a conclusion a release carries did, and where. The order it ran in is read off the one
// sequence every stage records into, because a count can say how often and never which came first.
struct conclusion
{
    int ran;
    bool on_its_own;
};

// Both release callables report into that sequence, so a case can say which of the two paths the
// composition took as well as what the conclusion did behind it.
preset_registry::factory composing_reporting(body_record &into, strand &own)
{
    return [&into, &own](const preset_site &site)
    {
        own = site.work;

        std::shared_ptr<preset> composed = compose(into, site);
        composed->release_cb             = [] { sequence.emplace_back("the acknowledgment ran"); };
        composed->release_fallback_cb    = [](bool concluding) { sequence.emplace_back(concluding ? "the fallback concluded" : "the fallback declined"); };

        return composed;
    };
}

// What each of a preset's two release callables did. It is written where a case still owns it once
// the stage has ended, because the verdict it pins is settled after that stage is gone.
struct release_record
{
    int acknowledged;
    int fell_back;
    bool unacknowledged;
};

preset_registry::factory composing_counting(body_record &into, release_record &kept)
{
    return [&into, &kept](const preset_site &site)
    {
        std::shared_ptr<preset> composed = compose(into, site);
        composed->release_cb             = [&kept] { ++kept.acknowledged; };
        composed->release_fallback_cb    = [&kept](bool unacknowledged)
        {
            ++kept.fell_back;
            kept.unacknowledged = unacknowledged;
        };

        return composed;
    };
}

praxis::detail::move_only_function<void()> concluding(conclusion &seen, const strand &own)
{
    return [&seen, own]
    {
        seen.on_its_own = own.running_here();
        ++seen.ran;
        sequence.emplace_back("the conclusion ran");
    };
}

struct frame_record
{
    int frames;
    int windows_added;
    int windows_removed;
};

// The frame with the display taken out of it: the body is rendered where the renderer renders it.
// The window list is the preset's own business, so what it registers arrives on the routes the stage
// installed rather than here.
void draw(composition &composed, frame_record &seen)
{
    ++seen.frames;

    if(composed.loaded())
        composed.composed()->stencil->render();
}

struct retirement
{
    bool acknowledged;
    bool on_the_strand;
};

retirement retire(scheduler &loop, const strand &on)
{
    retirement seen{false, false};

    REQUIRE(loop.retire_strand(on,
                               [&seen, on]
                               {
                                   seen.on_the_strand = on.running_here();
                                   seen.acknowledged  = true;
                               })
                    .has_value());
    REQUIRE(loop.drain().has_value());

    return seen;
}

// One headless stage: the scene a composition is built against, the strand its frames are posted
// to, and what those frames reported. The drain is what services them, so a case runs on the
// calling thread alone with no sleep, no retry and no wall clock in it.
struct stage
{
    stage()
            : loop(inline_workers, dictating())
            , scene(threepp::Scene::create())
            , work(*loop.make_strand())
            , body{}
            , seen{}
            , asked()
            , composed(*scene, loop, {})
    {
        sequence.clear();
        composed.unload_through([this] { composed.unload(); });
        composed.windows_through(
                [this](const window_share &)
                {
                    ++seen.windows_added;
                    sequence.emplace_back("a window registered");
                },
                [this](const window_share &)
                {
                    ++seen.windows_removed;
                    sequence.emplace_back("a window withdrawn");
                });
    }

    void draw_frame()
    {
        REQUIRE(work.post([this] { draw(composed, seen); }).has_value());
        REQUIRE(loop.drain().has_value());
    }

    scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    strand work;
    body_record body;
    frame_record seen;
    std::function<void()> asked;
    composition composed;
};

struct at_the_request
{
    bool loaded;
    bool released;
    bool alive;
};

// The unload asked for and carried out inside one posted handler, reporting what was already true
// when that handler returned: a drain would service the retirement acknowledgment before a case
// could look.
at_the_request unload_in_a_frame(stage &headless, const bool &released, const std::weak_ptr<owned_state> &owned)
{
    at_the_request seen{true, true, false};

    REQUIRE(headless.work
                    .post(
                            [&headless, &released, &owned, &seen]
                            {
                                headless.composed.unload();
                                seen.loaded   = headless.composed.loaded();
                                seen.released = released;
                                seen.alive    = !owned.expired();
                            })
                    .has_value());
    REQUIRE(headless.loop.drain().has_value());

    return seen;
}

}

TEST_CASE("a preset composes against a scene alone and is in place where the load was asked for", "[scene][composition]")
{
    stage headless;

    REQUIRE(headless.composed.load(composing(headless.body, headless.asked)).has_value());

    REQUIRE(headless.body.initialized == 1);
    REQUIRE(headless.seen.windows_added == 1);
    REQUIRE(headless.scene->children.size() == 1);

    headless.draw_frame();

    REQUIRE(headless.body.rendered == 1);
}

TEST_CASE("a builder holding no target and a builder answering nothing are both refused", "[scene][composition]")
{
    stage headless;

    const praxis::expected<void, load_refusal> empty = headless.composed.load(preset_registry::factory{});

    REQUIRE_FALSE(empty.has_value());
    REQUIRE(empty.error() == load_refusal::no_factory);

    const praxis::expected<void, load_refusal> silent = headless.composed.load([](const preset_site &) { return std::shared_ptr<preset>(); });

    REQUIRE_FALSE(silent.has_value());
    REQUIRE(silent.error() == load_refusal::refused);
    REQUIRE_FALSE(headless.composed.loaded());
}

TEST_CASE("a load while one composition is held switches to it, and the strand it left behind acknowledges on itself", "[scene][composition]")
{
    stage headless;
    body_record second{};
    acknowledgment left{};
    std::function<void()> unused;

    REQUIRE(headless.composed.load(composing_acknowledging(headless.body, left, "the first body")).has_value());
    REQUIRE(headless.composed.load(composing(second, unused, "the second body")).has_value());

    REQUIRE(headless.composed.loaded());
    REQUIRE(headless.scene->children.size() == 1);
    REQUIRE(sequence ==
            std::vector<std::string>{"the first body initialized", "a window registered", "a window withdrawn", "the first body torn down", "the second body initialized",
                                     "a window registered"});
    REQUIRE_FALSE(left.ran);

    REQUIRE(headless.loop.drain().has_value());

    REQUIRE(left.ran);
    REQUIRE(left.on_its_own);
    REQUIRE_FALSE(left.on_the_render_strand);
    REQUIRE(headless.body.destroyed);
}

TEST_CASE("a builder answering nothing while one composition is held leaves that one exactly as it was", "[scene][composition]")
{
    stage headless;
    strand made;

    REQUIRE(headless.composed.load(composing(headless.body, headless.asked)).has_value());

    const praxis::expected<void, load_refusal> silent = headless.composed.load(
            [&made](const preset_site &site)
            {
                made = site.work;

                return std::shared_ptr<preset>();
            });

    REQUIRE_FALSE(silent.has_value());
    REQUIRE(silent.error() == load_refusal::refused);
    REQUIRE(headless.composed.loaded());
    REQUIRE(headless.body.torn_down == 0);
    REQUIRE(headless.seen.windows_removed == 0);
    REQUIRE(headless.scene->children.size() == 1);

    const praxis::expected<void, rejection> late = made.post([] {});

    REQUIRE_FALSE(late.has_value());
    REQUIRE(late.error() == rejection::strand_retired);
}

TEST_CASE("a load a stopped scheduler can make no strand for is refused", "[scene][composition]")
{
    stage headless;

    headless.loop.stop();

    const praxis::expected<void, load_refusal> refused = headless.composed.load(composing(headless.body, headless.asked));

    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == load_refusal::no_strand);
    REQUIRE_FALSE(headless.composed.loaded());
}

TEST_CASE("the route a composition is given unloads it, and nothing of it outlives the unload", "[scene][composition]")
{
    stage headless;

    REQUIRE(headless.composed.load(composing(headless.body, headless.asked)).has_value());
    headless.draw_frame();
    REQUIRE(headless.asked != nullptr);

    // Asking records a handler; nothing of the scene graph moves where the request was made.
    headless.asked();

    REQUIRE(headless.composed.loaded());
    REQUIRE(headless.body.torn_down == 0);

    REQUIRE(headless.loop.drain().has_value());

    REQUIRE_FALSE(headless.composed.loaded());
    REQUIRE(headless.body.torn_down == 1);
    REQUIRE(headless.body.destroyed);
    REQUIRE(headless.seen.windows_removed == 1);
    REQUIRE(headless.scene->children.empty());
}

TEST_CASE("the strand a composition's frames run on retires behind the unload it already held", "[scene][composition]")
{
    stage headless;

    REQUIRE(headless.composed.load(composing(headless.body, headless.asked)).has_value());
    headless.draw_frame();

    headless.composed.unload();
    REQUIRE(headless.work.post([&headless] { draw(headless.composed, headless.seen); }).has_value());

    const retirement seen = retire(headless.loop, headless.work);

    REQUIRE(seen.acknowledged);
    REQUIRE(seen.on_the_strand);
    REQUIRE_FALSE(headless.composed.loaded());
    REQUIRE(headless.body.torn_down == 1);
    REQUIRE(headless.body.destroyed);

    const praxis::expected<void, rejection> late = headless.work.post([] {});

    REQUIRE_FALSE(late.has_value());
    REQUIRE(late.error() == rejection::strand_retired);
}

// The route names the strand the composition it came from was given, so one carried across an
// unload reaches a composition that is no longer held and unloads nothing.
TEST_CASE("a route carried over from a composition already unloaded leaves the next one alone", "[scene][composition]")
{
    stage headless;
    std::function<void()> carried;

    REQUIRE(headless.composed.load(composing(headless.body, carried)).has_value());
    headless.draw_frame();

    carried();
    REQUIRE(headless.loop.drain().has_value());
    REQUIRE_FALSE(headless.composed.loaded());
    REQUIRE(headless.body.torn_down == 1);

    REQUIRE(headless.composed.load(composing(headless.body, headless.asked)).has_value());
    headless.draw_frame();

    carried();
    REQUIRE(headless.loop.drain().has_value());

    REQUIRE(headless.composed.loaded());
    REQUIRE(headless.body.initialized == 2);
    REQUIRE(headless.body.torn_down == 1);

    // The control: what is left alone by the carried route is unloaded by its own.
    headless.asked();
    REQUIRE(headless.loop.drain().has_value());

    REQUIRE_FALSE(headless.composed.loaded());
    REQUIRE(headless.body.torn_down == 2);
}

TEST_CASE("the strand a composition's own state belongs to is never the one its frames are drawn on", "[scene][composition]")
{
    stage headless;
    const preset_registry::factory builder = composing(headless.body, headless.asked);

    strand_id own{};
    strand_id drawn{};

    REQUIRE(headless.composed
                    .load(
                            [&builder, &own, &drawn](const preset_site &site)
                            {
                                own   = site.work.id();
                                drawn = site.render.id();

                                return builder(site);
                            })
                    .has_value());

    REQUIRE(drawn == headless.loop.main_strand().id());
    REQUIRE(own != drawn);
    REQUIRE(own != headless.work.id());
    REQUIRE(headless.composed.composed()->work.id() == own);
}

TEST_CASE("a load whose builder answers nothing retires the strand that load made", "[scene][composition]")
{
    stage headless;
    strand made;

    const praxis::expected<void, load_refusal> silent = headless.composed.load(
            [&made](const preset_site &site)
            {
                made = site.work;

                return std::shared_ptr<preset>();
            });

    REQUIRE_FALSE(silent.has_value());
    REQUIRE(silent.error() == load_refusal::refused);
    REQUIRE(made.valid());

    const praxis::expected<void, rejection> late = made.post([] {});

    REQUIRE_FALSE(late.has_value());
    REQUIRE(late.error() == rejection::strand_retired);
}

TEST_CASE("what a composition put on its own strand is freed by the acknowledgment and not at the unload", "[scene][composition]")
{
    stage headless;
    bool released = false;
    std::weak_ptr<owned_state> owned;

    REQUIRE(headless.composed.load(composing_owning(headless.body, released, owned)).has_value());
    headless.draw_frame();

    REQUIRE_FALSE(owned.expired());

    const at_the_request seen = unload_in_a_frame(headless, released, owned);

    REQUIRE_FALSE(seen.loaded);
    REQUIRE_FALSE(seen.released);
    REQUIRE(seen.alive);
    REQUIRE(released);
    REQUIRE(owned.expired());
}

// The two cases below are one subject read from its two sides, and neither states it alone: which
// path a release takes decides where the conclusion it carries runs, and the discriminator is the
// drain. Here the conclusion is queued behind the acknowledgment and needs one; there the retirement
// is refused, nothing will ever be serviced, and the conclusion runs where it was asked for.
TEST_CASE("a conclusion a release carries runs behind the acknowledgment, on the strand it was retired on", "[scene][composition]")
{
    stage headless;
    conclusion seen{};
    strand own;

    REQUIRE(headless.composed.load(composing_reporting(headless.body, own)).has_value());
    headless.draw_frame();

    sequence.clear();
    headless.composed.release(concluding(seen, own));

    REQUIRE(seen.ran == 0);
    REQUIRE(sequence == std::vector<std::string>{"a window withdrawn", "the body torn down"});
    REQUIRE(headless.loop.drain().has_value());

    REQUIRE(seen.ran == 1);
    REQUIRE(seen.on_its_own);
    REQUIRE(sequence == std::vector<std::string>{"a window withdrawn", "the body torn down", "the acknowledgment ran", "the fallback declined", "the conclusion ran"});

    REQUIRE(headless.loop.drain().has_value());
    REQUIRE(seen.ran == 1);
}

TEST_CASE("a conclusion a stopped scheduler leaves no acknowledgment for runs where the release was asked for, once", "[scene][composition]")
{
    stage headless;
    conclusion seen{};
    conclusion again{};
    strand own;

    REQUIRE(headless.composed.load(composing_reporting(headless.body, own)).has_value());
    headless.draw_frame();
    headless.loop.stop();

    sequence.clear();
    headless.composed.release(concluding(seen, own));

    REQUIRE(seen.ran == 1);
    REQUIRE_FALSE(seen.on_its_own);
    REQUIRE(sequence == std::vector<std::string>{"a window withdrawn", "the body torn down", "the fallback concluded", "the conclusion ran"});

    const praxis::expected<void, rejection> serviced = headless.loop.drain();

    REQUIRE_FALSE(serviced.has_value());
    REQUIRE(seen.ran == 1);

    headless.composed.release(concluding(again, own));

    REQUIRE(again.ran == 1);
}

// The third verdict: the retirement is admitted, nothing ever services the strand, and the loop's own
// end discards what was queued there. Its settlement falls after the stage that records the sequence
// has ended, so what it did is counted where the case can still read it.
TEST_CASE("a conclusion nothing ever services runs where the strand's queued work is discarded, once", "[scene][composition]")
{
    release_record kept{};
    int concluded = 0;

    {
        stage headless;

        REQUIRE(headless.composed.load(composing_counting(headless.body, kept)).has_value());
        headless.draw_frame();
        headless.composed.release([&concluded] { ++concluded; });

        REQUIRE(kept.acknowledged == 0);
        REQUIRE(kept.fell_back == 0);
        REQUIRE(concluded == 0);
    }

    REQUIRE(kept.acknowledged == 0);
    REQUIRE(kept.fell_back == 1);
    REQUIRE(kept.unacknowledged);
    REQUIRE(concluded == 1);
}

TEST_CASE("the composition collects the windows of what it holds that carry settings, and lets them go with it", "[scene][composition]")
{
    stage headless;
    body_record second{};
    std::shared_ptr<settled_window> first;
    std::shared_ptr<settled_window> later;

    REQUIRE(headless.composed.configured().empty());
    REQUIRE(headless.composed.load(settling(headless.body, first, "windows.first", "the first body")).has_value());

    const std::vector<const praxis::config::configurable *> held = headless.composed.configured();

    REQUIRE(headless.seen.windows_added == 3);
    REQUIRE(held.size() == 1);
    REQUIRE(held.front()->settings_path() == "windows.first");
    REQUIRE(headless.composed.load(settling(second, later, "windows.second", "the second body")).has_value());

    const std::vector<const praxis::config::configurable *> replaced = headless.composed.configured();

    REQUIRE(replaced.size() == 1);
    REQUIRE(replaced.front()->settings_path() == "windows.second");

    headless.composed.unload();

    REQUIRE(first != nullptr);
    REQUIRE(later != nullptr);
    REQUIRE(headless.composed.configured().empty());
}

TEST_CASE("a window registered outside the preset's own list is collected and let go with the rest", "[scene][composition]")
{
    stage headless;
    window_route registering;
    std::shared_ptr<settled_window> listed;

    REQUIRE(headless.composed.load(offering(headless.body, listed, registering, "windows.listed")).has_value());

    const std::shared_ptr<settled_window> stray = std::make_shared<settled_window>("Stray", "windows.stray");

    REQUIRE(registering != nullptr);

    registering(stray);

    const std::vector<const praxis::config::configurable *> both = headless.composed.configured();

    REQUIRE(both.size() == 2);
    REQUIRE(both.front()->settings_path() == "windows.listed");
    REQUIRE(both.back()->settings_path() == "windows.stray");

    headless.composed.unload();

    REQUIRE(stray != nullptr);
    REQUIRE(headless.composed.configured().empty());
}
