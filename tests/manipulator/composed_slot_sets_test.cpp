#include "two_link_arm.h"

#include "praxis/manipulator/slots.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/scene/preset.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <vector>
#include <cstdint>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

// A scene is created headlessly and a renderer robot needs no graphics context, so no display is
// involved.
struct stage
{
    explicit stage(scheduler::scheduler &loop)
            : scene(threepp::Scene::create())
            , site{*scene, loop.main_strand(), *loop.make_strand(), [] {}, [](const std::shared_ptr<scene::imgui_window> &) {}, [](const std::shared_ptr<scene::imgui_window> &) {}, {}}
    {
    }

    std::shared_ptr<threepp::Scene> scene;
    scene::preset_site site;
};

struct seen
{
    forward_kinematics_slot_set fk;
    differential_kinematics_slot_set dk;
};

seen observe(stage &built, const capabilities &arm)
{
    seen taken;
    const arm_window_composer capturing = [&taken](const arm_window_inputs &offered)
    {
        taken.fk = offered.fk_inert;
        taken.dk = offered.dk_inert;

        return std::vector<std::shared_ptr<scene::imgui_window>>{};
    };

    const std::shared_ptr<scene::preset> composed =
            compose_arm(well_formed_arm(), built.site, attached_models{}, arm, trajectory::baseline(), rigid_motion::baseline(), joint_vector{}, capturing);

    REQUIRE(composed != nullptr);

    return taken;
}

capabilities over(forward_kinematics_ops forward, differential_kinematics_ops differential)
{
    capabilities composed = baseline();
    composed.fk           = forward;
    composed.dk           = differential;

    return composed;
}

forward_kinematics_ops only(forward_kinematics_slot bound)
{
    const forward_kinematics_ops written = baseline().fk;
    forward_kinematics_ops held{};

    if(bound == forward_kinematics_slot::forward_kinematics)
        held.forward_kinematics = written.forward_kinematics;
    if(bound == forward_kinematics_slot::body_forward_kinematics)
        held.body_forward_kinematics = written.body_forward_kinematics;
    if(bound == forward_kinematics_slot::body_screws_from_space)
        held.body_screws_from_space = written.body_screws_from_space;

    return held;
}

differential_kinematics_ops only(differential_kinematics_slot bound)
{
    const differential_kinematics_ops written = baseline().dk;
    differential_kinematics_ops held{};

    if(bound == differential_kinematics_slot::space_jacobian)
        held.space_jacobian = written.space_jacobian;
    if(bound == differential_kinematics_slot::body_jacobian)
        held.body_jacobian = written.body_jacobian;

    return held;
}

forward_kinematics_slot_set unwritten_beside(stage &built, forward_kinematics_slot bound)
{
    return observe(built, over(only(bound), differential_kinematics_ops{})).fk;
}

differential_kinematics_slot_set unwritten_beside(stage &built, differential_kinematics_slot bound)
{
    return observe(built, over(forward_kinematics_ops{}, only(bound))).dk;
}

template<slot_enumeration Slot>
void names_every_slot(const basic_slot_set<Slot> &named)
{
    REQUIRE(!named.empty());

    for(std::uint32_t index = 0; index < static_cast<std::uint32_t>(Slot::count); ++index)
    {
        INFO("slot " << index);
        REQUIRE(named.contains(static_cast<Slot>(index)));
    }
}

template<slot_enumeration Slot>
void each_bound_alone(stage &built)
{
    for(std::uint32_t index = 0; index < static_cast<std::uint32_t>(Slot::count); ++index)
    {
        const auto bound = static_cast<Slot>(index);
        INFO("bound slot " << index);

        const basic_slot_set<Slot> unwritten = unwritten_beside(built, bound);

        REQUIRE(!unwritten.contains(bound));

        for(std::uint32_t other = 0; other < static_cast<std::uint32_t>(Slot::count); ++other)
            if(other != index)
            {
                INFO("slot " << other);
                REQUIRE(unwritten.contains(static_cast<Slot>(other)));
            }
    }
}

}

TEST_CASE("a composition binding neither family names every slot of both", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const seen taken = observe(built, over(forward_kinematics_ops{}, differential_kinematics_ops{}));

    names_every_slot(taken.fk);
    names_every_slot(taken.dk);
}

TEST_CASE("one slot bound leaves every other slot of its family named", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    each_bound_alone<forward_kinematics_slot>(built);
    each_bound_alone<differential_kinematics_slot>(built);
}

TEST_CASE("a composition binding every slot of both families names none", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const seen taken = observe(built, baseline());

    REQUIRE(taken.fk.empty());
    REQUIRE(taken.dk.empty());
}

// Whether a slot counts as written is which function it points at, so a member assigned praxis's own
// inert implementation and one nobody touched are the same state.
TEST_CASE("a slot handed praxis's own inert function reads as one left alone", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const forward_kinematics_ops spelled_forward{&inert::forward_kinematics, &inert::body_forward_kinematics, &inert::body_screws_from_space};
    const differential_kinematics_ops spelled_differential{&inert::space_jacobian, &inert::body_jacobian};

    const seen taken = observe(built, over(spelled_forward, spelled_differential));

    names_every_slot(taken.fk);
    names_every_slot(taken.dk);
}
