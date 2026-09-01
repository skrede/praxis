#include "six_axis_machine.h"

#include "praxis/manipulator/modeling.h"
#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/scene_robot.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/screw_chain_builder.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <meios/model.h>

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

// Lynch & Park, Modern Robotics, eq. (3.24): a revolute screw is (w, -w x q) for a point q on its
// axis. Here the axis is the world y through a point on the x axis.
screw_axis about_y(double x)
{
    screw_axis axis;
    axis << 0.0, 1.0, 0.0, 0.0, 0.0, x;

    return axis;
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

// One screw axis per actuated joint of the description, turning about y and reaching along z, which
// is a different arm from the one the reference derivation reads out of the same description.
expected<screw_chain, refusal> supplied_chain(const meios::model<> &model)
{
    const std::size_t actuated =
            static_cast<std::size_t>(std::count_if(model.joints.begin(), model.joints.end(), [](const meios::joint<> &axis) { return axis.kind == meios::joint_kind::revolute; }));

    std::vector<screw_axis> screws;
    for(std::size_t axis = 0u; axis < actuated; ++axis)
        screws.push_back(about_y(static_cast<double>(axis) * link_length));

    transform home = transform::Identity();
    home(2, 3)     = static_cast<double>(actuated) * link_length;

    return screw_chain(home, std::move(screws), bounds_over(static_cast<Eigen::Index>(actuated)));
}

capabilities deriving_the_supplied_chain()
{
    capabilities arm = baseline();
    arm.modeling     = modeling_ops{.build_chain = &supplied_chain};

    return arm;
}

transform implied_by(const screw_chain &chain, const joint_vector &q)
{
    const expected<transform, refusal> pose = baseline().fk.forward_kinematics(chain.home, chain.space_screws, q);
    REQUIRE(pose.has_value());

    return pose.value();
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

    std::shared_ptr<threepp::Scene> scene;
    scene::preset_site site;
};

transform posed_at(scheduler::scheduler &loop, const std::weak_ptr<owned_arm> &driven, const joint_vector &q)
{
    expected<transform, refusal> reached = praxis::unexpected(refusal::not_implemented);
    command(driven,
            [&reached, &q](robot_controller &, scene_robot &machine)
            {
                machine.set_joint_positions(q);
                reached = machine.flange_pose();
            });
    REQUIRE(loop.drain().has_value());
    REQUIRE(reached.has_value());

    return reached.value();
}

}

TEST_CASE("the supplied chain builder is the one the composed machine's forward map follows", "[manipulator][preset]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    stage built(loop);

    const meios::model<> machine                   = six_axis_machine();
    const expected<screw_chain, refusal> supplied  = supplied_chain(machine);
    const expected<screw_chain, refusal> reference = build_screw_chain(machine);
    REQUIRE(supplied.has_value());
    REQUIRE(reference.has_value());

    std::weak_ptr<owned_arm> driven;
    const arm_window_composer capturing = [&driven](const arm_window_inputs &offered)
    {
        driven = offered.arm;

        return std::vector<std::shared_ptr<scene::imgui_window>>{};
    };

    const std::shared_ptr<scene::preset> composed =
            compose_arm(machine, built.site, attached_models{}, deriving_the_supplied_chain(), trajectory::baseline(), rigid_motion::baseline(), folded(), capturing);
    REQUIRE(composed != nullptr);
    REQUIRE(composed->initialize().has_value());
    REQUIRE_FALSE(driven.expired());

    const transform at_folded = posed_at(loop, driven, folded());
    REQUIRE(is_approx_equal(at_folded, implied_by(supplied.value(), folded()), 1.0e-9));
    REQUIRE_FALSE(is_approx_equal(at_folded, implied_by(reference.value(), folded()), 1.0e-6));

    const transform at_outstretched = posed_at(loop, driven, outstretched());
    REQUIRE(is_approx_equal(at_outstretched, implied_by(supplied.value(), outstretched()), 1.0e-9));
    REQUIRE_FALSE(is_approx_equal(at_outstretched, implied_by(reference.value(), outstretched()), 1.0e-6));

    composed->tear_down();
}

TEST_CASE("the supplied chain builder and the reference one answer differently for the same description", "[manipulator][preset]")
{
    const meios::model<> machine                   = six_axis_machine();
    const expected<screw_chain, refusal> supplied  = supplied_chain(machine);
    const expected<screw_chain, refusal> reference = build_screw_chain(machine);

    REQUIRE(supplied.has_value());
    REQUIRE(reference.has_value());
    REQUIRE(supplied.value().joint_count() == reference.value().joint_count());
    REQUIRE_FALSE(is_approx_equal(implied_by(supplied.value(), folded()), implied_by(reference.value(), folded()), 1.0e-6));
    REQUIRE_FALSE(is_approx_equal(implied_by(supplied.value(), outstretched()), implied_by(reference.value(), outstretched()), 1.0e-6));
}
