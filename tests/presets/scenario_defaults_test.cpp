#include "praxis/presets/arm.h"

#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/control_mode.h"
#include "praxis/manipulator/ik_seed_window.h"
#include "praxis/manipulator/tool_jog_window.h"
#include "praxis/manipulator/ik_branch_window.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/screw_jog_window.h"
#include "praxis/manipulator/ik_iterate_window.h"
#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/task_space_window.h"
#include "praxis/manipulator/world_object_window.h"
#include "praxis/manipulator/joint_control_window.h"
#include "praxis/manipulator/ik_convergence_window.h"
#include "praxis/manipulator/control_parameters_window.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <meios/urdf/load.h>

#include <Eigen/Core>

#include <cstddef>

using namespace praxis;

TEST_CASE("each control window's settings defaults to the state the shipped machines open at", "[presets][configuration]")
{
    REQUIRE(manipulator::joint_control_window::settings().mode == manipulator::control_mode::simulation);
    REQUIRE(manipulator::tool_jog_window::settings().mode == manipulator::control_mode::simulation);
    REQUIRE(manipulator::screw_jog_window::settings().mode == manipulator::control_mode::simulation);
    REQUIRE(manipulator::task_space_window::settings().shape == manipulator::task_space_window::motion_shape::ptp);
    REQUIRE(manipulator::task_space_window::settings().mode == manipulator::control_mode::simulation);

    const manipulator::tool_window::settings tool;
    REQUIRE_FALSE(tool.active);
    REQUIRE(tool.model_path.empty());
    REQUIRE(tool.selected_view == manipulator::tool_window::tool_view::load_stl);
    REQUIRE(tool.gfx_euler_degrees.isZero());
    REQUIRE(tool.gfx_euler_order == axis_order::zyx);
    REQUIRE(tool.gfx_scale.isApprox(Eigen::Vector3f::Ones()));
    REQUIRE(tool.gfx_offset.isZero());
    REQUIRE(tool.kinematics_euler_degrees.isZero());
    REQUIRE(tool.kinematics_euler_order == axis_order::zyx);
    REQUIRE(tool.kinematics_offset.isZero());

    const manipulator::world_object_window::settings world;
    REQUIRE_FALSE(world.active);
    REQUIRE(world.model_path.empty());
    REQUIRE(world.selected_view == manipulator::world_object_window::world_view::load_stl);
    REQUIRE(world.gfx_scale.isApprox(Eigen::Vector3f::Ones()));
    REQUIRE(world.gfx_offset.isZero());
    REQUIRE(world.gfx_euler_zyx_degrees.isZero());

    const manipulator::robot_view_window::settings view;
    REQUIRE(view.model == manipulator::model_render::meshes);
    REQUIRE(view.decoration);
    REQUIRE_FALSE(view.axis_reach.has_value());

    REQUIRE(manipulator::control_parameters_window::settings().velocity == Catch::Approx(0.3f));
}

TEST_CASE("each inverse-kinematics window's settings defaults to the state the shipped machines open at", "[presets][configuration]")
{
    // A shipped document names no start, so the list opens at the spread the joint count answers
    // rather than at a list somebody had to write out.
    REQUIRE(manipulator::ik_seed_window::settings().seeds.empty());
    REQUIRE(manipulator::ik_seed_window::opening_seeds(6u).size() == 8u);
    REQUIRE(manipulator::ik_seed_window::opening_seeds(6u).front().isZero());

    const manipulator::ik_branch_window::settings branches;
    REQUIRE(branches.mode == manipulator::control_mode::simulation);
    REQUIRE(branches.figures);

    const manipulator::ik_iterate_window::settings steps;
    REQUIRE(steps.start == 0u);
    REQUIRE(steps.mode == manipulator::control_mode::simulation);

    const manipulator::ik_convergence_window::settings falling;
    REQUIRE(falling.angular);
    REQUIRE(falling.linear);
}

TEST_CASE("an arm scenario written with no initializer holds a default in every member it carries", "[presets][configuration]")
{
    presets::arm_scenario chosen;

    REQUIRE(chosen.options.on_missing == meios::missing_asset::fail);
    REQUIRE(chosen.description.empty());
    REQUIRE(chosen.initial.size() == 0);

    REQUIRE_FALSE(chosen.tool.active);
    REQUIRE(chosen.tool.model_path.empty());
    REQUIRE(chosen.tool.selected_view == manipulator::tool_window::tool_view::load_stl);
    REQUIRE(chosen.tool.gfx_euler_degrees.isZero());
    REQUIRE(chosen.tool.gfx_euler_order == axis_order::zyx);
    REQUIRE(chosen.tool.gfx_scale.isApprox(Eigen::Vector3f::Ones()));
    REQUIRE(chosen.tool.gfx_offset.isZero());
    REQUIRE(chosen.tool.kinematics_euler_degrees.isZero());
    REQUIRE(chosen.tool.kinematics_euler_order == axis_order::zyx);
    REQUIRE(chosen.tool.kinematics_offset.isZero());

    REQUIRE_FALSE(chosen.recording.active);
    REQUIRE(chosen.recording.directory.empty());

    REQUIRE(chosen.tool_jog.mode == manipulator::control_mode::simulation);
    REQUIRE(chosen.screw_jog.mode == manipulator::control_mode::simulation);
    REQUIRE(chosen.task_space.shape == manipulator::task_space_window::motion_shape::ptp);
    REQUIRE(chosen.task_space.mode == manipulator::control_mode::simulation);

    REQUIRE_FALSE(chosen.world_object.active);
    REQUIRE(chosen.world_object.model_path.empty());
    REQUIRE(chosen.world_object.selected_view == manipulator::world_object_window::world_view::load_stl);
    REQUIRE(chosen.world_object.gfx_scale.isApprox(Eigen::Vector3f::Ones()));
    REQUIRE(chosen.world_object.gfx_offset.isZero());
    REQUIRE(chosen.world_object.gfx_euler_zyx_degrees.isZero());

    REQUIRE(chosen.robot_view.model == manipulator::model_render::meshes);
    REQUIRE(chosen.robot_view.decoration);
    REQUIRE_FALSE(chosen.robot_view.axis_reach.has_value());

    REQUIRE(chosen.joint_control.mode == manipulator::control_mode::simulation);
    REQUIRE(chosen.parameters.velocity == Catch::Approx(0.3f));

    REQUIRE(chosen.ik_seeds.seeds.empty());
    REQUIRE(chosen.ik_branch.mode == manipulator::control_mode::simulation);
    REQUIRE(chosen.ik_branch.figures);
    REQUIRE(chosen.ik_iterates.start == 0u);
    REQUIRE(chosen.ik_iterates.mode == manipulator::control_mode::simulation);
    REQUIRE(chosen.ik_convergence.angular);
    REQUIRE(chosen.ik_convergence.linear);
}
