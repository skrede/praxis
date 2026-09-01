#include "two_link_arm.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/stencil.h"
#include "praxis/scene/composition.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/presets/screw.h"
#include "praxis/presets/two_pose.h"
#include "praxis/presets/twist_axis.h"
#include "praxis/presets/euler_rungs.h"
#include "praxis/presets/rotation_axis.h"
#include "praxis/presets/frame_workbench.h"

#include "praxis/compat/expected.h"

#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scheduler/strand.h"
#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>
#include <threepp/core/Object3D.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;
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

manipulator::joint_vector one_joint(double at)
{
    manipulator::joint_vector q(1);
    q << at;

    return q;
}

manipulator::arm_window_composer one_window()
{
    return [](const manipulator::arm_window_inputs &) { return std::vector<window_share>{std::make_shared<silent_window>("Panel")}; };
}

// The binding a project that has written nothing below the scene graph composes with: the arm is
// drawn and no capability drives it.
manipulator::capabilities without_the_chain_derivation()
{
    manipulator::capabilities arm = manipulator::baseline();
    arm.modeling                  = manipulator::modeling_ops{};

    return arm;
}

std::shared_ptr<preset> arm_over(const preset_site &site, const manipulator::capabilities &arm)
{
    return manipulator::compose_arm(well_formed_arm(), site, manipulator::attached_models{}, arm, trajectory::baseline(), rigid_motion::baseline(), one_joint(0.25), one_window());
}

struct shipped_preset
{
    std::string name;
    preset_registry::factory build;
};

// One entry per preset factory the extensions declare, and one more for the partial binding
// that composes the same arm undriven. It is written out rather than taken from the list below so
// that dropping an entry there fails here instead of quietly narrowing every assertion in the file.
constexpr std::size_t shipped_factories = 9;

// Every preset the library ships, and beside them the partial binding that composes the same arm
// undriven. Nothing below reads a name of its own, so the only way for a preset to be exempt from
// what this file asserts is to be absent from this list.
std::vector<shipped_preset> shipped_presets()
{
    return {{"One frame beside a fixed one", [](const preset_site &site) { return presets::euler_rung_preset(site, rigid_motion::baseline(), presets::euler_rung::single_frame); }},
            {"Two frames beside a fixed one", [](const preset_site &site) { return presets::euler_rung_preset(site, rigid_motion::baseline(), presets::euler_rung::paired_frames); }},
            {"A frame tree built while it runs", [](const preset_site &site) { return presets::frame_workbench_preset(site, rigid_motion::baseline()); }},
            {"Screw motion: parameters", [](const preset_site &site) { return presets::screw_preset(site, rigid_motion::baseline()); }},
            {"The axis a twist names", [](const preset_site &site) { return presets::twist_axis_preset(site, rigid_motion::baseline()); }},
            {"The screw between two poses", [](const preset_site &site) { return presets::two_pose_preset(site, rigid_motion::baseline()); }},
            {"A frame turned about an axis through it", [](const preset_site &site) { return presets::rotation_axis_preset(site, rigid_motion::baseline()); }},
            {"Arm the reference solver drives", [](const preset_site &site) { return arm_over(site, manipulator::baseline()); }},
            {"Arm shown and not driven", [](const preset_site &site) { return arm_over(site, without_the_chain_derivation()); }}};
}

preset_registry populated()
{
    preset_registry into;
    for(shipped_preset &entry : shipped_presets())
        into.register_preset(entry.name, std::move(entry.build));

    return into;
}

std::vector<std::string> sorted_names(std::vector<std::string> names)
{
    std::sort(names.begin(), names.end());

    return names;
}

std::vector<std::string> registered_names()
{
    std::vector<std::string> names;
    for(const shipped_preset &entry : shipped_presets())
        names.push_back(entry.name);

    return sorted_names(std::move(names));
}

// One headless stage: the scene a composition is built against, the window list its routes record
// into, and the scheduler whose drain is the only thing that services anything. The clock is
// dictated and never advanced, so a preset that admits a sampled task is never due and no case
// waits on a wall clock.
struct stage
{
    stage()
            : loop(inline_workers, dictating())
            , scene(threepp::Scene::create())
            , shown()
            , held(*scene, loop, {})
    {
        held.unload_through([this] { held.unload(); });
        held.windows_through([this](const window_share &panel) { shown.push_back(panel); },
                             [this](const window_share &panel) { shown.erase(std::find(shown.begin(), shown.end(), panel)); });
    }

    std::size_t descendants()
    {
        std::size_t counted = 0;
        scene->traverse([&counted](threepp::Object3D &) { ++counted; });

        return counted;
    }

    praxis::scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    // Declared ahead of the composition so that it is destroyed after one: what the composition
    // holds is released at its destruction, and the withdrawal reaches this list.
    std::vector<window_share> shown;
    composition held;
};

// A stand-in for whatever a preset's release callable carries. The wrapper below owns that callable
// and drops it in the same statement it drops this, so this expiring is those shares being freed.
struct mark
{
    int carried;
};

struct observed
{
    std::weak_ptr<mark> freed;
    std::weak_ptr<preset> composed;
    std::weak_ptr<stencil> body;
    std::vector<std::weak_ptr<imgui_window>> panels;
};

struct acknowledgment
{
    bool ran;
    bool on_its_own;
    bool on_the_render_strand;
};

// Puts an acknowledgment of this file's own around whatever the preset composed for itself, so one
// instrument reads both where the retirement's acknowledgment runs and when the shares that
// callable carries are dropped.
observed watch(stage &built, acknowledgment &seen)
{
    const std::shared_ptr<preset> &composed = built.held.composed();
    auto freed                              = std::make_shared<mark>();

    observed watched{freed, composed, composed->stencil, {}};
    for(const window_share &panel : composed->windows)
        watched.panels.emplace_back(panel);

    composed->release_cb = [&seen, own = composed->work, render = built.loop.main_strand(), carried = std::move(composed->release_cb), held = std::move(freed)]() mutable
    {
        if(carried != nullptr)
            carried();
        carried = nullptr;
        held.reset();

        seen.on_its_own           = own.running_here();
        seen.on_the_render_strand = render.running_here();
        seen.ran                  = true;
    };

    return watched;
}

void require_nothing_left(const observed &watched)
{
    REQUIRE(watched.freed.expired());
    REQUIRE(watched.composed.expired());
    REQUIRE(watched.body.expired());
    for(const std::weak_ptr<imgui_window> &panel : watched.panels)
        REQUIRE(panel.expired());
}

std::map<std::string, std::size_t> extent_of_each(preset_registry &shipped)
{
    std::map<std::string, std::size_t> counted;
    for(const std::string &name : shipped.preset_names())
    {
        stage built;

        REQUIRE(built.held.load(shipped.load_preset(name)).has_value());
        counted.emplace(name, built.descendants());

        built.held.unload();
        REQUIRE(built.loop.drain().has_value());
    }

    return counted;
}

}

TEST_CASE("the registry enumerates exactly the names it was given and nothing else", "[presets][registry]")
{
    const preset_registry nothing;

    REQUIRE(nothing.preset_names().empty());

    preset_registry shipped                   = populated();
    const std::vector<std::string> enumerated = sorted_names(shipped.preset_names());

    REQUIRE(enumerated == registered_names());
    REQUIRE(enumerated.size() == registered_names().size());
    REQUIRE(enumerated.size() == shipped_factories);
    REQUIRE(shipped_presets().size() == shipped_factories);

    for(const std::string &name : enumerated)
        REQUIRE(shipped.load_preset(name) != nullptr);

    REQUIRE(shipped.load_preset("a name nothing was registered under") == nullptr);
}

// A registry is an instance a consuming project owns rather than a process-wide table, so two of
// them enumerate what each was given and neither sees the other's.
TEST_CASE("two registries enumerate independently", "[presets][registry]")
{
    preset_registry mine;
    preset_registry yours;

    mine.register_preset("a label only one registry was given", [](const preset_site &) { return std::shared_ptr<preset>(); });

    REQUIRE(mine.preset_names().size() == 1u);
    REQUIRE(yours.preset_names().empty());
    REQUIRE(yours.load_preset("a label only one registry was given") == nullptr);
}

// What a name registered twice does, recorded rather than prescribed: the second registration
// replaces the first, the enumeration answers one name, and the builder reached is the second.
TEST_CASE("a name registered twice leaves one entry, and it is the second registration", "[presets][registry]")
{
    preset_registry twice;
    bool first_asked  = false;
    bool second_asked = false;

    twice.register_preset("the same label",
                          [&first_asked](const preset_site &)
                          {
                              first_asked = true;

                              return std::shared_ptr<preset>();
                          });
    twice.register_preset("the same label",
                          [&second_asked](const preset_site &)
                          {
                              second_asked = true;

                              return std::shared_ptr<preset>();
                          });

    REQUIRE(twice.preset_names() == std::vector<std::string>{"the same label"});

    stage built;
    const expected<void, load_refusal> refused = built.held.load(twice.load_preset("the same label"));

    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == load_refusal::refused);
    REQUIRE_FALSE(first_asked);
    REQUIRE(second_asked);
}

// The two failure classes an unload is measured by, and neither catches the other: a node left in
// the graph is what the descendant count reads, and an object left owned is what the weak observers
// read. Both are asserted over every preset the registry enumerates.
TEST_CASE("a load and an unload leave nothing behind, over every preset the registry enumerates", "[presets][lifetime]")
{
    preset_registry shipped = populated();

    for(const std::string &name : shipped.preset_names())
    {
        CAPTURE(name);

        stage built;
        acknowledgment seen{};

        const std::size_t before = built.descendants();

        REQUIRE(built.held.load(shipped.load_preset(name)).has_value());
        REQUIRE(built.held.loaded());
        REQUIRE(built.descendants() > before);

        const observed watched = watch(built, seen);

        built.held.unload();

        REQUIRE_FALSE(built.held.loaded());
        REQUIRE(built.descendants() == before);
        REQUIRE(built.shown.empty());

        // What the release callable carries is still where it was: the acknowledgment the drain
        // below services is what frees it, and no handler already queued can reach past it.
        REQUIRE_FALSE(watched.freed.expired());
        REQUIRE_FALSE(seen.ran);

        REQUIRE(built.loop.drain().has_value());

        REQUIRE(seen.ran);
        REQUIRE(seen.on_its_own);
        REQUIRE_FALSE(seen.on_the_render_strand);
        require_nothing_left(watched);
        REQUIRE(built.descendants() == before);
    }
}

// A switch is the same withdrawal an unload takes, so it is measured with the same two instruments:
// what the retired preset left owned, and what the scene is left holding, which is what the preset
// switched to would have left it holding alone.
TEST_CASE("a load over a preset already held releases that one, for every ordered pair", "[presets][lifetime]")
{
    preset_registry shipped                        = populated();
    const std::vector<std::string> names           = shipped.preset_names();
    const std::map<std::string, std::size_t> alone = extent_of_each(shipped);

    for(const std::string &first : names)
        for(const std::string &second : names)
        {
            if(first == second)
                continue;

            CAPTURE(first, second);

            stage built;
            acknowledgment seen{};

            REQUIRE(built.held.load(shipped.load_preset(first)).has_value());

            const observed watched = watch(built, seen);

            REQUIRE(built.held.load(shipped.load_preset(second)).has_value());

            REQUIRE(built.held.loaded());
            REQUIRE(built.descendants() == alone.at(second));
            REQUIRE_FALSE(watched.freed.expired());
            REQUIRE_FALSE(seen.ran);

            REQUIRE(built.loop.drain().has_value());

            REQUIRE(seen.ran);
            REQUIRE(seen.on_its_own);
            REQUIRE_FALSE(seen.on_the_render_strand);
            require_nothing_left(watched);
            REQUIRE(built.held.loaded());
            REQUIRE(built.descendants() == alone.at(second));
        }
}
