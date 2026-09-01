#include "praxis/scene/preset.h"
#include "praxis/scene/stencil.h"
#include "praxis/scene/composition.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>
#include <threepp/core/Object3D.hpp>

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cstddef>
#include <utility>

using namespace praxis;
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

constexpr step_period ticking{seconds{0.01}};

// What the body and the two routes did, kept outside the preset so a case reads it without holding a
// share of anything the preset owns. The two window lists are names in the order they arrived, and
// `order` interleaves the two with a sign. `among` is what each body found already in the scene at
// the moment it placed itself.
struct site_record
{
    std::vector<std::string> added;
    std::vector<std::string> removed;
    std::vector<std::string> order;
    std::vector<std::size_t> among;
    int initialized;
    int torn_down;
    int torn_down_unplaced;
};

std::size_t descendants(threepp::Object3D &root)
{
    std::size_t counted = 0;
    root.traverse([&counted](threepp::Object3D &) { ++counted; });

    return counted;
}

class recording_body : public stencil
{
public:
    recording_body(site_record &into, threepp::Scene &target, bool refusing)
            : m_refusing(refusing)
            , m_placed(false)
            , m_into(into)
            , m_target(target)
            , m_node(threepp::Object3D::create())
    {
    }

    expected<void, refusal> initialize() override
    {
        if(m_refusing)
            return praxis::unexpected(refusal::not_implemented);

        m_into.among.push_back(descendants(m_target));
        ++m_into.initialized;
        m_target.add(m_node);
        m_placed = true;

        return {};
    }

    // m_placed is never cleared, so a body torn down twice counts as unplaced neither time.
    void tear_down() override
    {
        ++m_into.torn_down;
        if(!m_placed)
            ++m_into.torn_down_unplaced;

        m_target.remove(*m_node);
    }

    void render() const override
    {
    }

private:
    bool m_refusing;
    bool m_placed;
    site_record &m_into;
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

window_route adding(site_record &into)
{
    return [&into](const window_share &panel)
    {
        into.added.push_back(panel->display_name());
        into.order.push_back("+" + panel->display_name());
    };
}

window_route removing(site_record &into)
{
    return [&into](const window_share &panel)
    {
        into.removed.push_back(panel->display_name());
        into.order.push_back("-" + panel->display_name());
    };
}

std::vector<window_share> two_panels()
{
    return {std::make_shared<silent_window>("First"), std::make_shared<silent_window>("Second")};
}

std::shared_ptr<preset> compose(site_record &into, threepp::Scene &target, std::vector<window_share> panels, bool refusing = false)
{
    return std::make_shared<preset>(std::make_shared<recording_body>(into, target, refusing), std::move(panels), adding(into), removing(into));
}

// A stand-in for whatever a composition puts on the strand it is given. A case observes it through a
// weak pointer, and the only share left is the one the release route carries.
struct owned_state
{
    int touched;
};

// What a case watches the whole composition through. Every one of them is weak, so nothing here
// keeps anything of the preset alive.
struct observed
{
    std::weak_ptr<preset> composed;
    std::weak_ptr<stencil> body;
    std::weak_ptr<imgui_window> panel;
    std::weak_ptr<owned_state> driven;
};

preset_registry::factory composing(site_record &into, observed &watched, bool refusing = false)
{
    return [&into, &watched, refusing](const preset_site &site)
    {
        auto driven = std::make_shared<owned_state>();

        std::shared_ptr<preset> built = std::make_shared<preset>(std::make_shared<recording_body>(into, site.scene, refusing), two_panels(), site.add_window, site.remove_window);
        built->release_cb             = [held = driven]() mutable { held.reset(); };

        watched = observed{built, built->stencil, built->windows.front(), driven};

        return built;
    };
}

preset_registry::factory refusing_admission(site_record &into, observed &watched)
{
    return [&into, &watched](const preset_site &site)
    {
        std::shared_ptr<preset> built = composing(into, watched)(site);
        built->admit_cb               = [] { return praxis::unexpected(refusal::no_solution); };

        return built;
    };
}

preset_registry::factory asked(std::size_t &at_request, preset_registry::factory answering)
{
    return [&at_request, answering](const preset_site &site)
    {
        at_request = descendants(site.scene);

        return answering == nullptr ? nullptr : answering(site);
    };
}

void advance(praxis::scheduler::scheduler &loop, seconds by)
{
    dictated += std::chrono::duration_cast<time_point::duration>(by);
    REQUIRE(loop.drain().has_value());
}

}

TEST_CASE("a preset puts its own body and windows in place and takes them out in reverse", "[scene][preset]")
{
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    site_record seen{};

    const std::size_t before                = descendants(*target);
    const std::shared_ptr<preset> displayed = compose(seen, *target, two_panels());

    REQUIRE(displayed->initialize().has_value());

    REQUIRE(seen.initialized == 1);
    REQUIRE(descendants(*target) == before + 1);
    REQUIRE(seen.added == std::vector<std::string>{"First", "Second"});

    displayed->tear_down();

    REQUIRE(seen.torn_down == 1);
    REQUIRE(descendants(*target) == before);
    REQUIRE(seen.removed == std::vector<std::string>{"Second", "First"});
}

TEST_CASE("a preset carrying no window and no steppable is neither refused nor withdrawn from", "[scene][preset]")
{
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    site_record seen{};

    const std::shared_ptr<preset> bare = compose(seen, *target, std::vector<window_share>{});

    REQUIRE(bare->initialize().has_value());
    bare->tear_down();

    REQUIRE(seen.initialized == 1);
    REQUIRE(seen.torn_down == 1);
    REQUIRE(seen.added.empty());
    REQUIRE(seen.removed.empty());
}

TEST_CASE("a preset whose body refuses is refused before a window route is reached", "[scene][preset]")
{
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    site_record seen{};

    const std::size_t before              = descendants(*target);
    const std::shared_ptr<preset> refused = compose(seen, *target, two_panels(), true);
    const expected<void, refusal> placed  = refused->initialize();

    REQUIRE_FALSE(placed.has_value());
    REQUIRE(placed.error() == refusal::not_implemented);
    REQUIRE(descendants(*target) == before);
    REQUIRE(seen.added.empty());
    REQUIRE(seen.removed.empty());
    REQUIRE(seen.torn_down == 0);
}

TEST_CASE("a preset whose steppables are refused unwinds the steps that had succeeded", "[scene][preset]")
{
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    site_record seen{};

    const std::size_t before              = descendants(*target);
    const std::shared_ptr<preset> refused = compose(seen, *target, two_panels());

    refused->admit_cb = [] { return praxis::unexpected(refusal::no_solution); };

    const expected<void, refusal> placed = refused->initialize();

    REQUIRE_FALSE(placed.has_value());
    REQUIRE(placed.error() == refusal::no_solution);
    REQUIRE(descendants(*target) == before);
    REQUIRE(seen.added == std::vector<std::string>{"First", "Second"});
    REQUIRE(seen.removed == std::vector<std::string>{"Second", "First"});
    REQUIRE(seen.torn_down == 1);
}

TEST_CASE("the steppables a preset admitted stop ticking at its teardown and what they drove does not", "[scene][preset]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    site_record seen{};

    auto driven                                = std::make_shared<owned_state>();
    const std::weak_ptr<owned_state> observing = driven;
    owned_state &reached                       = *driven;

    const strand own                      = *loop.make_strand();
    const std::shared_ptr<preset> stepped = compose(seen, *target, std::vector<window_share>{});

    stepped->work       = own;
    stepped->release_cb = [held = std::move(driven)]() mutable { held.reset(); };
    stepped->admit_cb   = [own, &reached]
    {
        std::vector<task_handle> admitted;
        admitted.push_back(own.every(ticking, overrun::catch_up, [&reached](step_delta) { ++reached.touched; }));

        return expected<std::vector<task_handle>, refusal>(std::move(admitted));
    };

    REQUIRE(stepped->initialize().has_value());
    REQUIRE(stepped->steppables.size() == 1);

    advance(loop, seconds{0.05});
    const int ran = reached.touched;

    REQUIRE(ran > 0);

    stepped->tear_down();

    REQUIRE(stepped->steppables.empty());
    REQUIRE_FALSE(observing.expired());

    advance(loop, seconds{0.05});

    REQUIRE(reached.touched == ran);
    REQUIRE_FALSE(observing.expired());

    REQUIRE(loop.retire_strand(own, std::move(stepped->release_cb)).has_value());
    REQUIRE(loop.drain().has_value());

    REQUIRE(observing.expired());
}

TEST_CASE("nothing of a loaded preset outlives the acknowledgment of its strand's retirement", "[scene][preset]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    site_record seen{};
    observed watched;
    composition held(*target, loop, {});

    held.windows_through(adding(seen), removing(seen));

    const std::size_t before = descendants(*target);

    REQUIRE(held.load(composing(seen, watched)).has_value());

    REQUIRE(descendants(*target) == before + 1);
    REQUIRE(seen.added == std::vector<std::string>{"First", "Second"});

    held.unload();

    REQUIRE(descendants(*target) == before);
    REQUIRE(seen.removed == std::vector<std::string>{"Second", "First"});

    // The aggregate the release route carries is still where it was: the acknowledgment the drain
    // below services is what frees it.
    REQUIRE_FALSE(watched.driven.expired());

    REQUIRE(loop.drain().has_value());

    REQUIRE(watched.composed.expired());
    REQUIRE(watched.body.expired());
    REQUIRE(watched.panel.expired());
    REQUIRE(watched.driven.expired());
    REQUIRE(descendants(*target) == before);
}

TEST_CASE("nothing of the preset a load switched away from outlives that load's acknowledgment", "[scene][preset]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    site_record seen{};
    observed watched;
    observed next;
    composition held(*target, loop, {});

    held.windows_through(adding(seen), removing(seen));

    REQUIRE(held.load(composing(seen, watched)).has_value());

    const std::size_t alone = descendants(*target);

    REQUIRE(held.load(composing(seen, next)).has_value());

    // What the retired composition's release route carries is still where it was: the acknowledgment
    // the drain below services is what frees it.
    REQUIRE_FALSE(watched.driven.expired());
    REQUIRE(descendants(*target) == alone);
    REQUIRE(seen.removed == std::vector<std::string>{"Second", "First"});

    REQUIRE(loop.drain().has_value());

    REQUIRE(watched.composed.expired());
    REQUIRE(watched.body.expired());
    REQUIRE(watched.panel.expired());
    REQUIRE(watched.driven.expired());
    REQUIRE(descendants(*target) == alone);
    REQUIRE_FALSE(next.composed.expired());
}

TEST_CASE("a composition whose admission refuses leaves the scene as it found it", "[scene][preset]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    site_record seen{};
    observed watched;
    composition held(*target, loop, {});

    held.windows_through(adding(seen), removing(seen));

    const std::size_t before = descendants(*target);

    REQUIRE(held.load(refusing_admission(seen, watched)).has_value());

    REQUIRE_FALSE(held.loaded());
    REQUIRE(descendants(*target) == before);
    REQUIRE(seen.added == std::vector<std::string>{"First", "Second"});

    // The refusal is unwound twice: once where it is raised, and once more by the release the
    // composition reaches for afterwards.
    REQUIRE(seen.removed == std::vector<std::string>{"Second", "First", "Second", "First"});
    REQUIRE(seen.torn_down == 2);
    REQUIRE(seen.torn_down_unplaced == 0);
}

TEST_CASE("a composition whose body refuses tears that body down with nothing having been placed", "[scene][preset]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    site_record seen{};
    observed watched;
    composition held(*target, loop, {});

    held.windows_through(adding(seen), removing(seen));

    const std::size_t before = descendants(*target);

    REQUIRE(held.load(composing(seen, watched, true)).has_value());

    REQUIRE_FALSE(held.loaded());
    REQUIRE(seen.initialized == 0);
    REQUIRE(seen.added.empty());
    REQUIRE(descendants(*target) == before);
    REQUIRE(seen.torn_down == 1);
    REQUIRE(seen.torn_down_unplaced == 1);

    // The release withdraws windows that were never registered, having reached it past the body's
    // refusal rather than through a placement.
    REQUIRE(seen.removed == std::vector<std::string>{"Second", "First"});
}

TEST_CASE("a replacing load leaves the scene alone until the replacement initializes", "[scene][preset]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    site_record seen{};
    observed watched;
    observed next;
    composition held(*target, loop, {});

    held.windows_through(adding(seen), removing(seen));

    const std::size_t bare = descendants(*target);

    REQUIRE(held.load(composing(seen, watched)).has_value());

    const std::size_t alone = descendants(*target);
    std::size_t at_request  = 0;

    SECTION("a builder answering nothing leaves what was held exactly where it was")
    {
        const expected<void, load_refusal> answered = held.load(asked(at_request, nullptr));

        REQUIRE_FALSE(answered.has_value());
        REQUIRE(answered.error() == load_refusal::refused);
        REQUIRE(at_request == alone);
        REQUIRE(held.loaded());
        REQUIRE(held.composed() == watched.composed.lock());
        REQUIRE(descendants(*target) == alone);
        REQUIRE(seen.order == std::vector<std::string>{"+First", "+Second"});
    }

    SECTION("a builder answering a preset is asked before what was held is withdrawn")
    {
        REQUIRE(held.load(asked(at_request, composing(seen, next))).has_value());

        REQUIRE(at_request == alone);
        REQUIRE(descendants(*target) == alone);

        // Both bodies found the scene equally empty when they placed themselves, so the two never
        // stood in it together.
        REQUIRE(seen.among == std::vector<std::size_t>{bare, bare});
        REQUIRE(seen.order == std::vector<std::string>{"+First", "+Second", "-Second", "-First", "+First", "+Second"});
    }
}
