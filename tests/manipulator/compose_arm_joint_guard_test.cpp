#include "captured_log.h"
#include "two_link_arm.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/scene_robot.h"
#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <meios/model.h>

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

meios::joint<> revolute_joint(std::string name, std::string parent, std::string child)
{
    meios::joint<> record{};
    record.name   = std::move(name);
    record.kind   = meios::joint_kind::revolute;
    record.parent = std::move(parent);
    record.child  = std::move(child);
    record.axis   = {0.0, 0.0, 1.0};

    return record;
}

// Two revolute joints rather than one, so a configuration one joint short of this arm is a
// configuration of length one and not the empty one the two cases would otherwise share.
meios::model<> elbowed_arm(std::string name)
{
    meios::model<> model;
    model.name   = std::move(name);
    model.links  = {link_named("base"), link_named("upper"), link_named("tool")};
    model.joints = {revolute_joint("shoulder", "base", "upper"), revolute_joint("elbow", "upper", "tool")};
    model.topo   = meios::robot_topology{{-1, 0, 1}, {-1, 0, 1}, {0}, {0, 1, 2}};

    return model;
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

// The holder on its own, reached without a composition, which is how what the setter does with a
// length it was not built for is read directly.
scene_robot bare_holder(std::uint32_t joints)
{
    const auto counted = static_cast<Eigen::Index>(joints);
    screw_chain chain(transform::Identity(), std::vector<screw_axis>(joints, screw_axis::Zero()), bounds_over(counted));

    const capabilities arm = baseline();

    return scene_robot::compose(kinematics::compose(std::move(chain), arm.fk, arm.dk, arm.ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).value(), arm.robot,
                                rigid_motion::baseline().frame, joints)
            .value();
}

joint_vector two_joints(double first, double second)
{
    joint_vector q(2);
    q << first, second;

    return q;
}

// A scene is created headlessly and a renderer robot needs no graphics context, so no display is
// involved.
struct stage
{
    explicit stage(scheduler::scheduler &loop)
            : scene(threepp::Scene::create())
            , site{*scene, loop.main_strand(), *loop.make_strand(), [] {}, [](const std::shared_ptr<scene::imgui_window> &) {}, [](const std::shared_ptr<scene::imgui_window> &) {}, {}}
    {
    }

    std::size_t descendants()
    {
        std::size_t counted = 0;
        scene->traverse([&counted](threepp::Object3D &) { ++counted; });

        return counted;
    }

    std::shared_ptr<threepp::Scene> scene;
    scene::preset_site site;
};

struct opened
{
    std::shared_ptr<scene::preset> composed;
    std::weak_ptr<owned_arm> arm;
    std::string diagnosis;
};

opened open_named(stage &built, std::string name, const joint_vector &initial)
{
    opened taken;
    const arm_window_composer capturing = [&taken](const arm_window_inputs &offered)
    {
        taken.arm = offered.arm;

        return std::vector<std::shared_ptr<scene::imgui_window>>{};
    };

    captured_log captured;
    taken.composed  = compose_arm(elbowed_arm(std::move(name)), built.site, attached_models{}, baseline(), trajectory::baseline(), rigid_motion::baseline(), initial, capturing);
    taken.diagnosis = captured.text();

    return taken;
}

opened open_at(stage &built, const joint_vector &initial)
{
    return open_named(built, "elbowed_arm", initial);
}

// What the holder was actually given, read off the holder rather than off the rendered node, since
// it is the holder the setter was handed.
joint_vector opened_at(scheduler::scheduler &loop, const opened &taken)
{
    joint_vector held;
    command(taken.arm, [&held](robot_controller &, scene_robot &arm) { held = arm.joint_positions(); });
    REQUIRE(loop.drain().has_value());

    return held;
}

}

// What the guard rests on: the setter is a bare assignment, so a length the holder was not built for
// is taken, resized into and reported nowhere, leaving the count the holder answers and the count it
// holds disagreeing.
TEST_CASE("the holder takes an opening configuration of any length and refuses none of them", "[manipulator][preset]")
{
    scene_robot held = bare_holder(2);

    REQUIRE(held.joint_count() == 2u);
    REQUIRE(held.joint_positions().size() == 2);

    const std::string overlong = reported_by([&held] { held.set_joint_positions(joint_vector::Zero(5)); });

    CHECK(overlong.empty());
    CHECK(held.joint_count() == 2u);
    CHECK(held.joint_positions().size() == 5);

    const std::string emptied = reported_by([&held] { held.set_joint_positions(joint_vector{}); });

    CHECK(emptied.empty());
    CHECK(held.joint_count() == 2u);
    CHECK(held.joint_positions().size() == 0);
}

TEST_CASE("a scenario supplying no opening configuration opens the arm at zeros of its own joint count", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const opened taken = open_at(built, joint_vector{});

    REQUIRE(taken.composed != nullptr);
    REQUIRE(taken.diagnosis.empty());
    REQUIRE(taken.composed->initialize().has_value());

    const joint_vector held = opened_at(loop, taken);

    REQUIRE(held.size() == 2);
    CHECK(held.isZero());

    taken.composed->tear_down();
}

TEST_CASE("a scenario supplying an opening configuration of the arm's own length opens at it", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const joint_vector asked = two_joints(0.25, -0.5);
    const opened taken       = open_at(built, asked);

    REQUIRE(taken.composed != nullptr);
    REQUIRE(taken.diagnosis.empty());
    REQUIRE(taken.composed->initialize().has_value());

    const joint_vector held = opened_at(loop, taken);

    REQUIRE(held.size() == 2);
    CHECK(held.isApprox(asked));

    taken.composed->tear_down();
}

TEST_CASE("a scenario supplying an opening configuration one joint short composes nothing and names both counts", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const std::size_t before = built.descendants();
    const opened taken       = open_at(built, joint_vector::Zero(1));

    REQUIRE(taken.composed == nullptr);
    REQUIRE(built.descendants() == before);
    REQUIRE_THAT(taken.diagnosis,
                 Catch::Matchers::ContainsSubstring("'manipulator.compose_arm' refused the description of model 'elbowed_arm' as unsupported input; its opening "
                                                    "configuration has 1 joint values and the arm has 2 joints, so the preset is not composed"));
}

TEST_CASE("a scenario supplying an opening configuration one joint too many composes nothing and names both counts", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const std::size_t before = built.descendants();
    const opened taken       = open_at(built, joint_vector::Zero(3));

    REQUIRE(taken.composed == nullptr);
    REQUIRE(built.descendants() == before);
    REQUIRE_THAT(taken.diagnosis,
                 Catch::Matchers::ContainsSubstring("'manipulator.compose_arm' refused the description of model 'elbowed_arm' as unsupported input; its opening "
                                                    "configuration has 3 joint values and the arm has 2 joints, so the preset is not composed"));
}

// The name is read off the description the scenario supplied, so a second scenario refused the same
// way is told apart from the first by what the report says.
TEST_CASE("the refusal names the scenario it refused and not a fixed name", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const opened taken = open_named(built, "reaching_arm", joint_vector::Zero(3));

    REQUIRE(taken.composed == nullptr);
    REQUIRE_THAT(taken.diagnosis, Catch::Matchers::ContainsSubstring("model 'reaching_arm'"));
    REQUIRE_THAT(taken.diagnosis, !Catch::Matchers::ContainsSubstring("elbowed_arm"));
}
