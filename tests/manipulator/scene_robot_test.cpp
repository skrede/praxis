#include "fixtures.h"

#include "captured_log.h"

#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/scene_robot.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/scene_robot_builder.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <meios/model.h>

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <type_traits>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

transform shifted_tool_pose(const transform &pose, const transform &offset)
{
    return pose + offset;
}

Eigen::Vector3d doubled_position(const transform &pose)
{
    return 2.0 * pose.block<3, 1>(0, 3);
}

rotation negated_orientation(const transform &pose)
{
    return -pose.block<3, 3>(0, 0);
}

expected<joint_vector, refusal> shifted_solution(const kinematics &, const rigid_motion::frame_ops &, const transform &, const joint_vector &j0, const transform &)
{
    return joint_vector(j0 + joint_vector::Constant(j0.size(), 1.0));
}

expected<joint_vector, refusal> halved_solution(const kinematics &, const transform &, const joint_vector &j0)
{
    return joint_vector(0.5 * j0);
}

robot_ops filled_robot()
{
    return robot_ops{.tool_pose_from_flange_pose = &shifted_tool_pose,
                     .position_from_pose         = &doubled_position,
                     .orientation_from_pose      = &negated_orientation,
                     .ik_solve_pose              = &shifted_solution,
                     .ik_solve_flange_pose       = &halved_solution};
}

scene_robot adapter(const robot_ops &injected)
{
    return two_joint_arm(injected);
}

// The chain the renderer's arm has, with forward kinematics left unbound.
kinematics unsolved_arm()
{
    return kinematics::compose(sliding_chain(), forward_kinematics_ops{}, differential_kinematics_ops{}, inverse_kinematics_ops{}, rigid_motion::baseline().screw,
                               rigid_motion::baseline().frame)
            .value();
}

kinematics three_joint_solver()
{
    joint_limits bounds{};
    bounds.velocity       = joint_vector::Constant(3, 1.0);
    bounds.acceleration   = joint_vector::Constant(3, 4.0);
    bounds.lower_position = joint_vector::Constant(3, -1.5);
    bounds.upper_position = joint_vector::Constant(3, 1.5);

    const screw_chain chain(transform::Identity(), {screw_axis::Zero(), screw_axis::Zero(), screw_axis::Zero()}, bounds);

    return kinematics::compose(chain, forward_kinematics_ops{}, differential_kinematics_ops{}, inverse_kinematics_ops{}, rigid_motion::baseline().screw, rigid_motion::baseline().frame)
            .value();
}

meios::model<> link_less_arm()
{
    meios::model<> model;
    model.name = "link_less_arm";

    return model;
}

arm_window_composer no_windows()
{
    return [](const arm_window_inputs &) { return std::vector<std::shared_ptr<scene::imgui_window>>{}; };
}

// Both builders refuse the same malformed descriptions, so a null preset alone does not say which of
// them refused; the diagnosis is what distinguishes a composition that stopped at the scene graph
// from one that reached the chain derivation.
std::string composition_diagnosis(const meios::model<> &description, const scene::preset_site &site, std::shared_ptr<scene::preset> &composed)
{
    captured_log captured;

    composed = compose_arm(description, site, attached_models{}, baseline(), trajectory::baseline(), rigid_motion::baseline(), configuration(0.0, 0.0), no_windows());

    return captured.text();
}

meios::model<> rootless_arm()
{
    meios::link<> base;
    base.name = "base";
    meios::link<> shoulder;
    shoulder.name = "shoulder";

    meios::model<> model;
    model.name       = "rootless_arm";
    model.links      = {base, shoulder};
    model.topo.roots = {};

    return model;
}

}

// The written value is one no float represents, so a configuration that survived a conversion to
// single precision and back would fail the read exactly where the double one passes.
TEST_CASE("the_configuration_the_holder_reports_is_the_double_one_it_was_given")
{
    scene_robot robot = adapter(robot_ops{});

    REQUIRE(robot.joint_count() == 2u);
    CHECK(robot.joint_positions().isZero(default_tolerance));

    const double unrepresentable = 0.25 + 1.0e-12;
    robot.set_joint_positions(configuration(unrepresentable, -0.5));
    CHECK(robot.joint_positions()[0] == unrepresentable);
    CHECK(robot.joint_positions()[1] == -0.5);
}

TEST_CASE("the_joint_limits_come_from_the_chain_the_solver_carries")
{
    const scene_robot robot = adapter(robot_ops{});

    REQUIRE(robot.limits().lower_position.size() == 2);
    CHECK(is_approx_equal(robot.limits().lower_position[0], -1.5));
    CHECK(is_approx_equal(robot.limits().upper_position[1], 1.5));
}

TEST_CASE("the_tool_offset_is_the_adapters_own_state")
{
    scene_robot robot = adapter(robot_ops{});

    CHECK(is_approx_equal(robot.tool_offset(), transform::Identity()));

    transform offset = transform::Identity();
    offset(2, 3)     = 0.15;
    robot.set_tool_offset(offset);
    CHECK(is_approx_equal(robot.tool_offset(), offset));
}

TEST_CASE("the_flange_pose_is_the_solvers_forward_kinematics_at_the_held_configuration")
{
    scene_robot robot = adapter(robot_ops{});
    robot.set_joint_positions(configuration(0.4, 0.0));

    transform reached = transform::Identity();
    reached(0, 3)     = 0.4;
    CHECK(is_approx_equal(robot.flange_pose().value(), reached));
}

TEST_CASE("the_pose_at_a_supplied_configuration_is_the_held_one_when_the_held_one_is_supplied")
{
    scene_robot robot = adapter(filled_robot());
    robot.set_joint_positions(configuration(0.4, -0.2));

    transform offset = transform::Identity();
    offset(1, 3)     = 0.25;
    robot.set_tool_offset(offset);

    CHECK(is_approx_equal(robot.tool_pose_at(robot.joint_positions()).value(), robot.tool_pose().value()));
}

// Sampling a motion takes a pose at every one of its configurations, so an accessor that moved the
// arm to answer would leave it wherever the last sample stood.
TEST_CASE("the_pose_at_a_supplied_configuration_leaves_the_held_configuration_where_it_was")
{
    scene_robot robot = adapter(filled_robot());
    robot.set_joint_positions(configuration(0.4, -0.2));

    REQUIRE(robot.tool_pose_at(configuration(-0.9, 1.1)).has_value());
    CHECK(robot.joint_positions()[0] == 0.4);
    CHECK(robot.joint_positions()[1] == -0.2);
}

TEST_CASE("a_configuration_of_a_width_the_chain_does_not_have_is_refused_rather_than_thrown_on")
{
    const scene_robot robot                    = adapter(filled_robot());
    const expected<transform, refusal> refused = robot.tool_pose_at(joint_vector::Zero(3));

    REQUIRE_FALSE(refused.has_value());
    CHECK(refused.error() == refusal::unsupported_input);
}

// The pose accessors reach the solver first, so a solver that refuses forward kinematics is what
// every one of them reports -- with the solver's own enumerator, not one invented at the adapter.
TEST_CASE("a_solver_that_refuses_forward_kinematics_refuses_every_pose_the_adapter_reports")
{
    const scene_robot robot = scene_robot::compose(unsolved_arm(), filled_robot(), rigid_motion::baseline().frame, 2u).value();

    const expected<transform, refusal> flange   = robot.flange_pose();
    const expected<transform, refusal> tool     = robot.tool_pose();
    const expected<Eigen::Vector3d, refusal> at = robot.flange_position();
    const expected<rotation, refusal> turned    = robot.tool_orientation();

    REQUIRE_FALSE(flange.has_value());
    REQUIRE_FALSE(tool.has_value());
    REQUIRE_FALSE(at.has_value());
    REQUIRE_FALSE(turned.has_value());
    CHECK(flange.error() == refusal::not_implemented);
    CHECK(tool.error() == refusal::not_implemented);
    CHECK(at.error() == refusal::not_implemented);
    CHECK(turned.error() == refusal::not_implemented);
}

TEST_CASE("an_unbound_robot_capability_leaves_every_mathematics_accessor_inert")
{
    scene_robot robot = adapter(robot_ops{});
    robot.set_joint_positions(configuration(0.4, -0.2));

    CHECK(is_approx_equal(robot.tool_pose().value(), transform::Identity()));
    CHECK(robot.tool_position().value().isZero(default_tolerance));
    CHECK((robot.tool_orientation().value() - rotation::Identity()).isZero(default_tolerance));
    CHECK(robot.flange_position().value().isZero(default_tolerance));
    CHECK((robot.flange_orientation().value() - rotation::Identity()).isZero(default_tolerance));

    const joint_vector j0                             = configuration(0.1, 0.2);
    const expected<joint_vector, refusal> from_tool   = robot.ik_solve_pose(transform::Identity(), j0);
    const expected<joint_vector, refusal> from_flange = robot.ik_solve_flange_pose(transform::Identity(), j0);

    REQUIRE_FALSE(from_tool.has_value());
    REQUIRE_FALSE(from_flange.has_value());
    CHECK(from_tool.error() == refusal::not_implemented);
    CHECK(from_flange.error() == refusal::not_implemented);
}

TEST_CASE("every_mathematics_accessor_reports_what_the_injected_robot_slots_computed")
{
    scene_robot robot = adapter(filled_robot());
    robot.set_joint_positions(configuration(0.4, 0.0));

    transform offset = transform::Identity();
    offset(1, 3)     = 0.25;
    robot.set_tool_offset(offset);

    const transform flange = robot.flange_pose().value();
    const transform tool   = shifted_tool_pose(flange, offset);
    CHECK(is_approx_equal(robot.tool_pose().value(), tool));
    CHECK((robot.tool_position().value() - doubled_position(tool)).isZero(default_tolerance));
    CHECK((robot.tool_orientation().value() - negated_orientation(tool)).isZero(default_tolerance));
    CHECK((robot.flange_position().value() - doubled_position(flange)).isZero(default_tolerance));
    CHECK((robot.flange_orientation().value() - negated_orientation(flange)).isZero(default_tolerance));

    const joint_vector j0                             = configuration(0.6, -0.4);
    const expected<joint_vector, refusal> from_tool   = robot.ik_solve_pose(transform::Identity(), j0);
    const expected<joint_vector, refusal> from_flange = robot.ik_solve_flange_pose(transform::Identity(), j0);

    REQUIRE(from_tool.has_value());
    REQUIRE(from_flange.has_value());
    CHECK(is_approx_equal(*from_tool, joint_vector(j0 + joint_vector::Constant(2, 1.0))));
    CHECK(is_approx_equal(*from_flange, joint_vector(0.5 * j0)));
}

// A three-joint chain against the two-joint renderer handle: the two counts are genuinely different,
// so the case cannot pass on a holder that never compares them.
TEST_CASE("a_chain_of_a_length_the_rendered_arm_does_not_have_composes_no_holder")
{
    const auto composed = scene_robot::compose(three_joint_solver(), robot_ops{}, rigid_motion::baseline().frame, 2u);

    REQUIRE_FALSE(composed.has_value());
    REQUIRE(composed.error() == refusal::unsupported_input);
}

TEST_CASE("a_chain_of_the_length_the_rendered_arm_has_composes_a_holder_that_reports_that_length")
{
    const auto composed = scene_robot::compose(sliding_solver(), robot_ops{}, rigid_motion::baseline().frame, 2u);

    REQUIRE(composed.has_value());
    REQUIRE(composed->joint_count() == 2u);
}

// The composition is the only route to a holder, so a holder that never reconciled the two counts
// cannot be built at all. Move construction stays reachable, because the composition answers by
// value and the preset moves that value into the share it hands out.
TEST_CASE("the holder is reachable only through the composition that reconciles the two counts")
{
    STATIC_REQUIRE(!std::is_constructible_v<scene_robot, kinematics, const robot_ops &, const rigid_motion::frame_ops &>);
    STATIC_REQUIRE(std::is_move_constructible_v<scene_robot>);
}

TEST_CASE("a description carrying no link builds no scene graph")
{
    const auto built = build_scene_robot(link_less_arm());

    REQUIRE_FALSE(built.has_value());
    REQUIRE(built.error() == refusal::unsupported_input);
}

TEST_CASE("a description carrying no root link builds no scene graph")
{
    const auto built = build_scene_robot(rootless_arm());

    REQUIRE_FALSE(built.has_value());
    REQUIRE(built.error() == refusal::unsupported_input);
}

TEST_CASE("a composition the scene-graph builder refuses yields no preset and names that builder")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();

    std::shared_ptr<scene::preset> composed;
    const std::string diagnosis = composition_diagnosis(link_less_arm(), scene::preset_site{*target, loop.main_strand(), *loop.make_strand(), nullptr, nullptr, nullptr, {}}, composed);

    REQUIRE(composed == nullptr);
    REQUIRE(target->children.empty());
    REQUIRE_THAT(diagnosis, Catch::Matchers::ContainsSubstring("'manipulator.build_scene_robot' refused the description of model 'link_less_arm' as unsupported input"));
}
