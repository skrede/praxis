#ifndef HPP_GUARD_PRAXIS_PRESETS_ARM_H
#define HPP_GUARD_PRAXIS_PRESETS_ARM_H

#include "praxis/presets/screw_table.h"

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/ik_seed_window.h"
#include "praxis/manipulator/joint_curve_window.h"
#include "praxis/manipulator/tool_jog_window.h"
#include "praxis/manipulator/ik_branch_window.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/screw_jog_window.h"
#include "praxis/manipulator/ik_iterate_window.h"
#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/task_space_window.h"
#include "praxis/manipulator/edited_list_window.h"
#include "praxis/manipulator/world_object_window.h"
#include "praxis/manipulator/joint_control_window.h"
#include "praxis/manipulator/ik_convergence_window.h"
#include "praxis/manipulator/path_comparison_window.h"
#include "praxis/manipulator/render_controls_window.h"
#include "praxis/manipulator/waypoints_configuration.h"
#include "praxis/manipulator/control_parameters_window.h"
#include "praxis/manipulator/trajectory_preview_window.h"
#include "praxis/manipulator/velocity_kinematics_window.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include <meios/urdf/load.h>

#include <array>
#include <memory>
#include <cstddef>
#include <filesystem>

namespace praxis::presets {

// Where each of this scenario's settings-carrying windows keeps its values.
struct window_paths
{
    static constexpr const char *tool                = "tool";
    static constexpr const char *ik_seeds            = "ik_seeds";
    static constexpr const char *tool_jog            = "tool_jog";
    static constexpr const char *ik_branch           = "ik_branch";
    static constexpr const char *recording           = "recording";
    static constexpr const char *screw_jog           = "screw_jog";
    static constexpr const char *parameters          = "parameters";
    static constexpr const char *robot_view          = "robot_view";
    static constexpr const char *task_space          = "task_space";
    static constexpr const char *ik_iterates         = "ik_iterates";
    static constexpr const char *joint_curves        = "joint_curves";
    static constexpr const char *world_object        = "world_object";
    static constexpr const char *joint_control       = "joint_control";
    static constexpr const char *ik_convergence      = "ik_convergence";
    static constexpr const char *pose_waypoints      = "pose_waypoints";
    static constexpr const char *joint_waypoints     = "joint_waypoints";
    static constexpr const char *path_comparison     = "path_comparison";
    static constexpr const char *render_controls     = "render_controls";
    static constexpr const char *trajectory_preview  = "trajectory_preview";
    static constexpr const char *velocity_kinematics = "velocity_kinematics";

    // How many paths the struct names, stated here so that the table below is held to it.
    static constexpr std::size_t counted = 20;

    // Every path above, in the order they stand, so a caller checking that each of them is declared
    // and read walks a table instead of a list somebody kept in their head.
    static constexpr std::array every{tool,           ik_seeds,        tool_jog,        ik_branch,       recording,          screw_jog,          parameters,
                                      robot_view,     task_space,      ik_iterates,     world_object,    joint_curves,       joint_control,      ik_convergence,
                                      pose_waypoints, joint_waypoints, path_comparison, render_controls, trajectory_preview, velocity_kinematics};
};

static_assert(window_paths::every.size() == window_paths::counted);

// Everything one arm scenario composes from: what its description is loaded from, the joint values
// it starts at, and the state every window composed beside it opens in. No member names a document,
// a binding or a key, so a caller holding none of those composes the same scenario.
struct arm_scenario
{
    meios::load_options options;
    std::filesystem::path description;
    manipulator::joint_vector initial;
    manipulator::tool_window::settings tool;
    manipulator::recording_parameters recording;
    manipulator::tool_jog_window::settings tool_jog;
    manipulator::ik_seed_window::settings ik_seeds;
    manipulator::screw_jog_window::settings screw_jog;
    manipulator::ik_branch_window::settings ik_branch;
    manipulator::robot_view_window::settings robot_view;
    manipulator::task_space_window::settings task_space;
    manipulator::world_object_window::settings world_object;
    manipulator::ik_iterate_window::settings ik_iterates;
    manipulator::joint_curve_window::settings joint_curves;
    manipulator::joint_control_window::settings joint_control;
    manipulator::pose_waypoint_list::settings pose_waypoints;
    manipulator::joint_waypoint_list::settings joint_waypoints;
    manipulator::ik_convergence_window::settings ik_convergence;
    manipulator::control_parameters_window::settings parameters;
    manipulator::path_comparison_window::settings path_comparison;
    manipulator::render_controls_window::settings render_controls;
    manipulator::trajectory_preview_window::settings trajectory_preview;
    manipulator::velocity_kinematics_window::settings velocity_kinematics;
};

manipulator::arm_composition arm_windows(arm_scenario chosen);

// Joint values drive the arm, the pose window reads back what the bound forward map computed, and the
// screw axes the description itself names are drawn on the arm they describe.
manipulator::arm_composition arm_windows_forward(arm_scenario chosen);

// The chain is supplied rather than derived: the axes drawn are the ones typed into the window, and
// `keeping` is where a table typed here is opened from and kept. The rendered arm stays reachable
// and the drawn axes carry no switch, so nobody is left with only their own claim on screen.
manipulator::arm_composition arm_windows_modeling(arm_scenario chosen, screw_table_source keeping);

// A tool and a world object on top of forward kinematics, with the tool attachable and detachable
// while the scenario runs. The four frame transformations a tool pose is read through answer the
// origin unrotated when nobody binds them, which is a pose an arm can genuinely be at, so a
// composition handed any of them unbound opens no window at all and names the ones it was denied.
manipulator::arm_composition arm_windows_tooling(arm_scenario chosen);

// A target pose, a list of starts to search from and the answers a search found, with the steps of
// one start as a table and as a falling curve beside it. The solve happens where it is asked for and
// nowhere else, and every start the list carries is run.
manipulator::arm_composition arm_windows_numerical_ik(arm_scenario chosen);

// A list of waypoints a learner authors, run as a sequence of separate motions coming fully to rest
// at each of them. The motion is previewed before it plays -- its path drawn beside the one the tool
// last traversed, its path parameter scrubbed, and the scaling chosen for it plotted against the two
// it was chosen over -- and choosing another scaling changes both what plays and what is plotted.
manipulator::arm_composition arm_windows_point_to_point(arm_scenario chosen);

// The same kind of list, run as one motion passing through every waypoint without coming to rest at
// any of them. The motion is previewed before it plays, and beside the preview each joint's angle,
// its rate and the rate's own rate are drawn against time, which is where the rate carrying through
// a waypoint rather than returning to zero there is something a learner can see.
manipulator::arm_composition arm_windows_via_point(arm_scenario chosen);

// One pair of configurations and the three paths that join them -- the tool's own straight line, the
// same line with its turn coupled to it, and the line a straight run in joint values draws -- shown
// at once and each shown or hidden on its own. The ends are configurations, so what a learner reads
// off the picture is the shape each path takes and not what a solver made of it, and any one of the
// three can be run with the path the tool traversed drawn beside the one it was commanded along.
manipulator::arm_composition arm_windows_path_comparison(arm_scenario chosen);

// The same target pose and the same list of answers, filled in one call rather than searched. There
// is no list of starts, no table of steps and no curve, because a closed form takes no start and
// records no step.
manipulator::arm_composition arm_windows_analytic_ik(arm_scenario chosen);

// Both Jacobians read as an aligned matrix and drawn as the twists their columns stand for, with the
// two manipulability ellipsoids at the tool beside them and the force reading reachable in place of
// the velocity one. The singular values of each block, its manipulability measure and its condition
// number are read off beside the matrix, and one control governs which Jacobian all of it is taken
// from. Lynch & Park, Modern Robotics, sections 5.1 and 5.4.
manipulator::arm_composition arm_windows_velocity_kinematics(arm_scenario chosen);

// Nothing is composed where the description does not load, and the failure is named. What the
// composition opens and what it draws are both the caller's, so one factory serves every scenario
// built on an arm and none of them is named here.
std::shared_ptr<scene::preset> arm_preset(const scene::preset_site &site, const manipulator::capabilities &arm, const trajectory::capabilities &shapes,
                                          const rigid_motion::capabilities &motions, const arm_scenario &chosen, const manipulator::arm_composition &composed);

}

#endif
