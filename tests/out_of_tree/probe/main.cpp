#include "shipped_extension.h"

#include "praxis/extension.h"

#include "praxis/presets/euler_rungs.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/stencil.h"
#include "praxis/scene/composition.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/scheduler/scheduler.h"

#include <threepp/scenes/Scene.hpp>
#include <threepp/core/Object3D.hpp>

#include <set>
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <iostream>
#include <string_view>

namespace outside {

struct pose
{
    double along;
    double across;
    double heading;
};

}

namespace outside::inert {

double unit_radius()
{
    return 1.0;
}

double zero_slip(const pose &)
{
    return 0.0;
}

pose stationary(double)
{
    return pose{0.0, 0.0, 0.0};
}

}

namespace outside {

struct wheel_ops
{
    double (*rolling_radius)()          = &inert::unit_radius;
    double (*slip_ratio)(const pose &p) = &inert::zero_slip;
    pose (*contact_pose)(double t)      = &inert::stationary;
};

enum class wheel_slot : std::uint32_t
{
    rolling_radius,
    slip_ratio,
    contact_pose,
    count
};

constexpr std::array wheel_descriptors{
        praxis::slot_descriptor{"wheel.rolling_radius", [](const void *value) -> bool { return static_cast<const wheel_ops *>(value)->rolling_radius == &inert::unit_radius; }},
        praxis::slot_descriptor{"wheel.slip_ratio", [](const void *value) -> bool { return static_cast<const wheel_ops *>(value)->slip_ratio == &inert::zero_slip; }},
        praxis::slot_descriptor{"wheel.contact_pose", [](const void *value) -> bool { return static_cast<const wheel_ops *>(value)->contact_pose == &inert::stationary; }},
};

static_assert(wheel_descriptors.size() == static_cast<std::size_t>(wheel_slot::count));

constexpr praxis::capability_descriptors<wheel_ops> described_wheels{"outside", wheel_descriptors};

using wheel_slot_set = praxis::basic_slot_set<wheel_slot>;

praxis::capability_view view_of(const wheel_ops &ops)
{
    return praxis::capability_view::of(ops, described_wheels);
}

double locked_slip(const pose &)
{
    return 1.0;
}

double wide_radius()
{
    return 4.0;
}

pose drifting(double t)
{
    return pose{t, 0.5 * t, 0.25};
}

const wheel_ops defaulted_wheels{};
const wheel_ops bound_wheels{.rolling_radius = &wide_radius, .slip_ratio = &locked_slip, .contact_pose = &drifting};

bool the_empty_set_contains_nothing()
{
    wheel_slot_set empty;

    return empty.empty() && !empty.contains(wheel_slot::rolling_radius) && empty.set(wheel_slot::count).empty() && !empty.contains(wheel_slot::count);
}

bool complementing_moves_between_the_empty_set_and_every_slot()
{
    wheel_slot_set every;
    wheel_slot_set complement_of_none = ~wheel_slot_set();
    std::size_t held                  = 0;

    for(std::uint32_t index = 0; index < static_cast<std::uint32_t>(wheel_slot::count); ++index)
    {
        every.set(static_cast<wheel_slot>(index));
        held += complement_of_none.contains(static_cast<wheel_slot>(index)) ? 1u : 0u;
    }

    return held == wheel_descriptors.size() && !every.empty() && (~every).empty();
}

bool a_value_initialized_capability_holds_every_default()
{
    const std::array<praxis::capability_view, 1> views{view_of(defaulted_wheels)};

    return praxis::count_defaults(views) == wheel_descriptors.size();
}

bool a_capability_bound_through_every_member_holds_none()
{
    const std::array<praxis::capability_view, 1> views{view_of(bound_wheels)};

    return praxis::count_defaults(views) == 0u;
}

bool overriding_one_member_frees_exactly_that_slot()
{
    wheel_ops ops{.slip_ratio = &locked_slip};
    const praxis::capability_view view = view_of(ops);
    const std::array<praxis::capability_view, 1> views{view};
    const std::size_t past = static_cast<std::size_t>(wheel_slot::count);

    return praxis::count_defaults(views) == wheel_descriptors.size() - 1u && !praxis::holds_default(view, static_cast<std::size_t>(wheel_slot::slip_ratio)) &&
            praxis::holds_default(view, static_cast<std::size_t>(wheel_slot::contact_pose)) && !praxis::holds_default(view, past) && praxis::slot_name(view, past).empty();
}

bool the_report_names_every_defaulted_slot_uniquely()
{
    const std::array<praxis::capability_view, 1> views{view_of(defaulted_wheels)};
    const std::vector<praxis::defaulted_slot> report = praxis::defaulted_slots(views);
    std::set<std::string_view> names;

    for(const praxis::defaulted_slot &entry : report)
    {
        if(entry.extension != "outside" || entry.slot.empty() || !names.insert(entry.slot).second)
        {
            return false;
        }
    }
    return report.size() == wheel_descriptors.size() && praxis::slot_name(views.front(), 0) == "wheel.rolling_radius";
}

std::string_view the_capability_reports_itself()
{
    if(!the_empty_set_contains_nothing())
        return "the empty slot set did not answer as empty";
    if(!complementing_moves_between_the_empty_set_and_every_slot())
        return "complementing did not move between the empty set and every slot";
    if(!a_value_initialized_capability_holds_every_default())
        return "the composition left at its defaults did not report every slot as defaulted";
    if(!a_capability_bound_through_every_member_holds_none())
        return "the composition bound through every member still reported a defaulted slot";
    if(!overriding_one_member_frees_exactly_that_slot())
        return "overriding one member did not free exactly that slot";
    if(!the_report_names_every_defaulted_slot_uniquely())
        return "the defaulted-slot report did not name every slot once";

    return {};
}

// What the probe observes of one composed preset: what it did to the scene, and what it read back
// through the capability it was composed over.
struct trace
{
    int placed;
    int withdrawn;
    int rendered;
    double radius;
    double heading;
};

class rolling_body : public praxis::scene::stencil
{
public:
    rolling_body(trace &into, const wheel_ops &ops, threepp::Scene &target)
            : m_into(into)
            , m_ops(ops)
            , m_target(target)
            , m_node(threepp::Object3D::create())
    {
    }

    praxis::expected<void, praxis::refusal> initialize() override
    {
        ++m_into.placed;
        m_target.add(m_node);

        return {};
    }

    void tear_down() override
    {
        ++m_into.withdrawn;
        m_target.remove(*m_node);
    }

    void render() const override
    {
        ++m_into.rendered;
        m_into.radius  = m_ops.rolling_radius();
        m_into.heading = m_ops.contact_pose(1.0).heading;
    }

private:
    trace &m_into;
    const wheel_ops &m_ops;
    threepp::Scene &m_target;
    std::shared_ptr<threepp::Object3D> m_node;
};

class gauge_window : public praxis::scene::imgui_window
{
public:
    explicit gauge_window(std::string name)
            : imgui_window(std::move(name))
    {
    }

    void render() override
    {
    }
};

using window_share = std::shared_ptr<praxis::scene::imgui_window>;

praxis::scene::preset_registry::factory composing(const wheel_ops &ops, trace &into)
{
    return [&ops, &into](const praxis::scene::preset_site &site)
    {
        std::vector<window_share> panels{std::make_shared<gauge_window>("Wheel")};

        return std::make_shared<praxis::scene::preset>(std::make_shared<rolling_body>(into, ops, site.scene), std::move(panels), site.add_window, site.remove_window);
    };
}

// One headless stage: no display, no window, and no wall clock in any of it. The window list belongs
// to whoever holds the composition, so the probe installs routes of its own and counts through them.
struct stage
{
    stage()
            : added(0)
            , removed(0)
            , loop(praxis::scheduler::inline_workers)
            , scene(threepp::Scene::create())
            , composed(*scene, loop, {})
    {
        composed.windows_through([this](const window_share &) { ++added; }, [this](const window_share &) { ++removed; });
    }

    int added;
    int removed;
    praxis::scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    praxis::scene::composition composed;
};

std::string_view placed(stage &headless, const praxis::scene::preset_registry::factory &builder, const trace &into)
{
    if(!headless.composed.load(builder).has_value())
        return "the composition refused the builder the registry answered with";
    if(headless.scene->children.size() != 1u || into.placed != 1)
        return "the body is not in the scene after the load";
    if(headless.added != 1 || headless.removed != 0)
        return "the window was not registered through the route the site carried";

    headless.composed.composed()->stencil->render();
    if(into.rendered != 1)
        return "the body was not rendered while it was in place";

    return {};
}

std::string_view withdrawn(stage &headless, const std::weak_ptr<praxis::scene::preset> &observed, const trace &into)
{
    headless.composed.unload();
    if(!headless.loop.drain().has_value())
        return "the scheduler refused to drain the retirement the unload asked for";
    if(!headless.scene->children.empty() || into.withdrawn != 1)
        return "the body is still in the scene after the unload";
    if(headless.removed != 1)
        return "the window was not withdrawn through the route the site carried";
    if(!observed.expired())
        return "the preset outlived the unload";

    return {};
}

// The whole of one preset's life outside this repository: composed by a builder the probe
// registered, put in place, drawn once, then taken away with nothing of it left behind.
std::string_view drive(praxis::scene::preset_registry &registry, const std::string &name, const trace &into)
{
    stage headless;
    const praxis::scene::preset_registry::factory builder = registry.load_preset(name);
    if(builder == nullptr)
        return "the registry answered no builder for the name it was given";

    const std::string_view put = placed(headless, builder, into);
    if(!put.empty())
        return put;

    const std::weak_ptr<praxis::scene::preset> observed = headless.composed.composed();

    return withdrawn(headless, observed, into);
}

// A defaulted-slot count says how many slots were bound and not which implementation answered, so
// the two compositions are told apart by what the preset read back through them.
std::string_view the_two_compositions_answer_differently(const trace &defaulted, const trace &bound)
{
    if(defaulted.radius == bound.radius)
        return "both compositions answered the same rolling radius";
    if(defaulted.heading == bound.heading)
        return "both compositions answered the same contact heading";

    return {};
}

// A scenario this repository ships, composed by a consumer that only links it: the preset target is
// public linkable surface, and what it composes is reachable the same way outside the build tree.
std::string_view the_shipped_arrangement_composes_outside_the_tree()
{
    const std::shared_ptr<threepp::Scene> target          = threepp::Scene::create();
    const praxis::scene::window_route unwired             = [](const window_share &) {};
    const praxis::scene::preset_site site                 = {*target, praxis::scheduler::strand{}, praxis::scheduler::strand{}, [] {}, unwired, unwired, {}};
    const std::shared_ptr<praxis::scene::preset> composed = praxis::presets::euler_rung_preset(site, praxis::rigid_motion::baseline(), praxis::presets::euler_rung::single_frame);

    if(composed == nullptr)
        return "the shipped arrangement composed nothing";
    if(composed->windows.empty())
        return "the shipped arrangement composed no window at all";

    return {};
}

std::string_view the_probe_composes_its_own_presets()
{
    praxis::scene::preset_registry registry;
    trace defaulted{};
    trace bound{};

    registry.register_preset("outside.wheel.defaulted", composing(defaulted_wheels, defaulted));
    registry.register_preset("outside.wheel.bound", composing(bound_wheels, bound));
    if(registry.preset_names().size() != 2u)
        return "the probe's own registry did not answer with the two names it was given";

    const std::string_view first = drive(registry, "outside.wheel.defaulted", defaulted);
    if(!first.empty())
        return first;

    const std::string_view second = drive(registry, "outside.wheel.bound", bound);
    if(!second.empty())
        return second;

    return the_two_compositions_answer_differently(defaulted, bound);
}

}

int main()
{
    std::string_view failed = outside::the_capability_reports_itself();
    if(failed.empty())
        failed = outside::the_probe_composes_its_own_presets();
    if(failed.empty())
        failed = outside::the_shipped_arrangement_composes_outside_the_tree();
    if(failed.empty())
        failed = outside::the_shipped_manipulator_reports_its_slots();
    if(failed.empty())
        return 0;

    std::cerr << "the out-of-tree probe failed: " << failed << '\n';

    return 1;
}
