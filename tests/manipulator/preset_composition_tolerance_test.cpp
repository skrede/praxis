#include "two_link_arm.h"

#include "captured_log.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/scene_robot.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <meios/model.h>

#include <threepp/scenes/Scene.hpp>

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <algorithm>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

// What one composition puts under the scene: the rendered arm, and the decoration root beside it
// rather than under it, so that hiding either leaves the other shown.
constexpr std::size_t composed_nodes = 2;

// The rendered arm the well-formed description builds has one joint, so every chain a case hands the
// composition is written against that count or deliberately against another.
joint_vector one_joint(double at)
{
    joint_vector q(1);
    q << at;

    return q;
}

joint_limits bounds_over(Eigen::Index joints)
{
    joint_limits bounds{};
    bounds.velocity       = joint_vector::Constant(joints, 1.0);
    bounds.acceleration   = joint_vector::Constant(joints, 4.0);
    bounds.lower_position = joint_vector::Constant(joints, -3.0);
    bounds.upper_position = joint_vector::Constant(joints, 3.0);

    return bounds;
}

expected<transform, refusal> sliding_forward_kinematics(const transform &, std::span<const screw_axis>, const joint_vector &theta)
{
    transform pose = transform::Identity();
    pose(0, 3)     = theta.size() > 0 ? theta[0] : 0.0;

    return pose;
}

// Limits carrying no entry, which is the one thing the solver's composition asks of the chain itself.
expected<screw_chain, refusal> limitless_chain(const meios::model<> &)
{
    return screw_chain(transform::Identity(), {screw_axis::Zero()}, joint_limits{});
}

expected<screw_chain, refusal> three_joint_chain(const meios::model<> &)
{
    return screw_chain(transform::Identity(), {screw_axis::Zero(), screw_axis::Zero(), screw_axis::Zero()}, bounds_over(3));
}

capabilities without_the_chain_derivation()
{
    capabilities arm = baseline();
    arm.modeling     = modeling_ops{};

    return arm;
}

capabilities deriving(expected<screw_chain, refusal> (*chain)(const meios::model<> &))
{
    capabilities arm = baseline();
    arm.modeling     = modeling_ops{.build_chain = chain};

    return arm;
}

// What a project that has written the forward map and nothing else composes with.
capabilities forward_kinematics_only()
{
    return capabilities{.fk = forward_kinematics_ops{.forward_kinematics = &sliding_forward_kinematics}};
}

capabilities forward_kinematics_over_a_derived_chain()
{
    capabilities arm = forward_kinematics_only();
    arm.modeling     = baseline().modeling;

    return arm;
}

meios::model<> link_less_arm()
{
    meios::model<> model;
    model.name = "link_less_arm";

    return model;
}

class silent_window : public scene::imgui_window
{
public:
    explicit silent_window(std::string name)
            : scene::imgui_window(std::move(name))
    {
    }

    void render() override
    {
    }
};

arm_window_composer no_windows()
{
    return [](const arm_window_inputs &) { return std::vector<std::shared_ptr<scene::imgui_window>>{}; };
}

arm_window_composer one_window()
{
    return [](const arm_window_inputs &) { return std::vector<std::shared_ptr<scene::imgui_window>>{std::make_shared<silent_window>("Only")}; };
}

// Everything a composition is offered, with the two window routes recording rather than drawing. A
// scene is created headlessly and a renderer robot needs no graphics context, so no display is
// involved.
struct stage
{
    explicit stage(scheduler::scheduler &loop)
            : scene(threepp::Scene::create())
            , shown()
            , site{*scene,
                   loop.main_strand(),
                   *loop.make_strand(),
                   [] {},
                   [this](const std::shared_ptr<scene::imgui_window> &panel) { shown.push_back(panel); },
                   [this](const std::shared_ptr<scene::imgui_window> &panel) { shown.erase(std::find(shown.begin(), shown.end(), panel)); },
                   {}}
    {
    }

    std::size_t descendants()
    {
        std::size_t counted = 0;
        scene->traverse([&counted](threepp::Object3D &) { ++counted; });

        return counted;
    }

    std::shared_ptr<threepp::Scene> scene;
    std::vector<std::shared_ptr<scene::imgui_window>> shown;
    scene::preset_site site;
};

struct composition
{
    std::shared_ptr<scene::preset> composed;
    std::string diagnosis;
};

composition compose_with(stage &built, const capabilities &arm, const trajectory::capabilities &shapes, const rigid_motion::capabilities &motions, const meios::model<> &description,
                         const arm_window_composer &windows)
{
    captured_log captured;

    std::shared_ptr<scene::preset> composed = compose_arm(description, built.site, attached_models{}, arm, shapes, motions, one_joint(0.25), windows);

    return composition{std::move(composed), captured.text()};
}

composition compose(stage &built, const capabilities &arm, const meios::model<> &description)
{
    return compose_with(built, arm, trajectory::baseline(), rigid_motion::baseline(), description, no_windows());
}

std::size_t lines_in(const std::string &text)
{
    return static_cast<std::size_t>(std::count(text.begin(), text.end(), '\n'));
}

const threepp::Robot &node_of(const scene::preset &composed)
{
    return dynamic_cast<const loadable_robot_stencil &>(*composed.stencil).robot();
}

bool arm_in_scene(stage &built, const scene::preset &composed)
{
    const threepp::Robot &node = node_of(composed);

    return std::any_of(built.scene->children.begin(), built.scene->children.end(), [&node](const threepp::Object3D *child) { return child == &node; });
}

std::string rendered(scheduler::scheduler &loop, const scene::preset &composed, std::uint32_t frames)
{
    captured_log captured;

    for(std::uint32_t drawn = 0; drawn < frames; ++drawn)
        REQUIRE(loop.main_strand().post([&composed] { composed.stencil->render(); }).has_value());
    REQUIRE(loop.drain().has_value());

    return captured.text();
}

}

TEST_CASE("a description the scene-graph builder refuses composes no preset at all", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const std::size_t before    = built.descendants();
    const composition attempted = compose(built, baseline(), link_less_arm());

    REQUIRE(attempted.composed == nullptr);
    REQUIRE(built.descendants() == before);
    REQUIRE_THAT(
            attempted.diagnosis,
            Catch::Matchers::ContainsSubstring("'manipulator.build_scene_robot' refused the description of model 'link_less_arm' as unsupported input; the preset is not composed"));
}

TEST_CASE("a chain derivation that refuses leaves a preset that loads, renders and unloads", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const std::size_t before   = built.descendants();
    const composition undriven = compose(built, without_the_chain_derivation(), well_formed_arm());

    REQUIRE(undriven.composed != nullptr);
    REQUIRE_THAT(undriven.diagnosis,
                 Catch::Matchers::ContainsSubstring(
                         "'manipulator.modeling.build_chain' refused the description of model 'well_formed_arm' as not implemented; the arm is shown and not driven"));

    REQUIRE(undriven.composed->initialize().has_value());
    REQUIRE(built.descendants() > before);
    REQUIRE(built.scene->children.size() == composed_nodes);
    REQUIRE(arm_in_scene(built, *undriven.composed));

    REQUIRE(rendered(loop, *undriven.composed, 1).empty());
    CHECK(is_approx_equal(static_cast<double>(node_of(*undriven.composed).getJointValue(0)), 0.25, 1.0e-6));

    undriven.composed->tear_down();

    REQUIRE(built.descendants() == before);
}

TEST_CASE("a composition whose limits do not cover every joint still composes and reports once", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const composition undriven = compose(built, deriving(&limitless_chain), well_formed_arm());

    REQUIRE(undriven.composed != nullptr);
    REQUIRE(lines_in(undriven.diagnosis) == 1u);
    REQUIRE_THAT(undriven.diagnosis,
                 Catch::Matchers::ContainsSubstring(
                         "'manipulator.kinematics.compose' refused the description of model 'well_formed_arm' as unsupported input; the arm is shown and not driven"));
    REQUIRE(undriven.composed->initialize().has_value());

    undriven.composed->tear_down();
}

TEST_CASE("a rendered joint count the chain disagrees with is reported once naming both counts", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const composition undriven = compose(built, deriving(&three_joint_chain), well_formed_arm());

    REQUIRE(undriven.composed != nullptr);
    REQUIRE(lines_in(undriven.diagnosis) == 1u);
    REQUIRE_THAT(undriven.diagnosis, Catch::Matchers::ContainsSubstring("the rendered arm has 1 joints and the solver's chain has 3, so the two cannot be driven as one"));
    REQUIRE(undriven.composed->initialize().has_value());

    undriven.composed->tear_down();
}

// The two instruments an unload is measured with: what the scene is left holding, and whether the
// shares the composition took have reached the retirement acknowledgment.
TEST_CASE("an undriven preset registers no window, admits no steppable and leaves nothing behind", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const std::size_t before = built.descendants();
    composition undriven     = compose_with(built, without_the_chain_derivation(), trajectory::baseline(), rigid_motion::baseline(), well_formed_arm(), one_window());

    REQUIRE(undriven.composed != nullptr);
    REQUIRE(undriven.composed->windows.empty());
    REQUIRE(undriven.composed->admit_cb == nullptr);
    REQUIRE(undriven.composed->initialize().has_value());
    REQUIRE(built.shown.empty());
    REQUIRE(undriven.composed->steppables.empty());

    undriven.composed->tear_down();

    REQUIRE(built.descendants() == before);

    const std::weak_ptr<scene::preset> retired = undriven.composed;
    const std::weak_ptr<scene::stencil> drawn  = undriven.composed->stencil;

    REQUIRE(loop.retire_strand(undriven.composed->work, std::move(undriven.composed->release_cb)).has_value());
    undriven.composed.reset();
    REQUIRE(loop.drain().has_value());

    REQUIRE(retired.expired());
    REQUIRE(drawn.expired());
}

TEST_CASE("a fully bound description composes the driven arm and the composer's windows", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const std::size_t before = built.descendants();
    const composition driven = compose_with(built, baseline(), trajectory::baseline(), rigid_motion::baseline(), well_formed_arm(), one_window());

    REQUIRE(driven.composed != nullptr);
    REQUIRE(driven.diagnosis.empty());
    REQUIRE(driven.composed->windows.size() == 1u);

    REQUIRE(driven.composed->initialize().has_value());
    REQUIRE(built.descendants() > before);
    REQUIRE(built.scene->children.size() == composed_nodes);
    REQUIRE(built.shown.size() == 1u);
    REQUIRE(driven.composed->steppables.empty());

    driven.composed->tear_down();

    REQUIRE(built.descendants() == before);
    REQUIRE(built.shown.empty());
}

TEST_CASE("an undriven preset reports its refusal once per composition and not once per frame", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const composition undriven = compose(built, without_the_chain_derivation(), well_formed_arm());

    REQUIRE(undriven.composed != nullptr);
    REQUIRE(lines_in(undriven.diagnosis) == 1u);
    REQUIRE(undriven.composed->initialize().has_value());

    REQUIRE(lines_in(rendered(loop, *undriven.composed, 10u)) == 0u);

    undriven.composed->tear_down();
}

// A case asserting only that the preset loaded cannot tell a tolerant composition from one that
// silently put an inert implementation behind a slot the project left unbound, so what is read here
// is which capability refused.
TEST_CASE("a project that has written the forward map and nothing else gets a preset it can load", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const std::size_t before   = built.descendants();
    const composition undriven = compose_with(built, forward_kinematics_only(), trajectory::capabilities{}, rigid_motion::capabilities{}, well_formed_arm(), no_windows());

    REQUIRE(undriven.composed != nullptr);
    REQUIRE(undriven.composed->initialize().has_value());
    REQUIRE(built.descendants() > before);
    REQUIRE(built.scene->children.size() == composed_nodes);
    REQUIRE(arm_in_scene(built, *undriven.composed));
    REQUIRE_THAT(undriven.diagnosis, Catch::Matchers::ContainsSubstring("'manipulator.modeling.build_chain' refused the description of model 'well_formed_arm' as not implemented"));

    undriven.composed->tear_down();
}

TEST_CASE("a driven arm composed over a partial binding answers a Jacobian with the unbound slot's refusal", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    std::weak_ptr<owned_arm> reached;
    const arm_window_composer capturing = [&reached](const arm_window_inputs &offered)
    {
        reached = offered.arm;

        return std::vector<std::shared_ptr<scene::imgui_window>>{};
    };

    const std::size_t before = built.descendants();
    const composition driven = compose_with(built, forward_kinematics_over_a_derived_chain(), trajectory::capabilities{}, rigid_motion::capabilities{}, well_formed_arm(), capturing);

    REQUIRE(driven.composed != nullptr);
    REQUIRE(driven.composed->initialize().has_value());
    REQUIRE(built.descendants() > before);
    REQUIRE(built.scene->children.size() == composed_nodes);
    REQUIRE(arm_in_scene(built, *driven.composed));
    REQUIRE_FALSE(reached.expired());

    bool answered    = true;
    refusal reported = refusal::no_solution;
    command(reached,
            [&answered, &reported](robot_controller &, scene_robot &arm)
            {
                const expected<jacobian, refusal> taken = arm.solver().body_jacobian(arm.joint_positions());
                answered                                = taken.has_value();
                if(!answered)
                    reported = taken.error();
            });
    REQUIRE(loop.drain().has_value());

    REQUIRE_FALSE(answered);
    CHECK(reported == refusal::not_implemented);

    driven.composed->tear_down();
}

// Which screw slots hold their defaults is a fact of the composed capability, and the composition
// is the only place that holds it: nothing below reads the aggregate, and a set handed in by hand
// would say whatever it was told. Both the stencil and every window built beside it are given the
// one the composition read off the capability it was handed.
TEST_CASE("a composition reads the screw slots left at their defaults off the capability it was handed", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    rigid_motion::screw_slot_set offered;
    const arm_window_composer capturing = [&offered](const arm_window_inputs &given)
    {
        offered = given.screw_inert;

        return std::vector<std::shared_ptr<scene::imgui_window>>{};
    };

    const composition unbound = compose_with(built, forward_kinematics_over_a_derived_chain(), trajectory::capabilities{}, rigid_motion::capabilities{}, well_formed_arm(), capturing);

    REQUIRE(unbound.composed != nullptr);
    CHECK(offered.contains(rigid_motion::screw_slot::matrix_exponential_screw));
    CHECK(offered.contains(rigid_motion::screw_slot::screw_axis_from_angular_linear));
    CHECK(dynamic_cast<const loadable_robot_stencil &>(*unbound.composed->stencil).inert_screw_slots().contains(rigid_motion::screw_slot::matrix_exponential_screw));

    unbound.composed->tear_down();

    const composition bound = compose_with(built, baseline(), trajectory::baseline(), rigid_motion::baseline(), well_formed_arm(), capturing);

    REQUIRE(bound.composed != nullptr);
    CHECK(offered.empty());
    CHECK(dynamic_cast<const loadable_robot_stencil &>(*bound.composed->stencil).inert_screw_slots().empty());

    bound.composed->tear_down();
}
