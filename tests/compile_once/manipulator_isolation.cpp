#include "praxis/extension.h"
#include "praxis/manipulator.h"

#if defined(HPP_GUARD_CARTAN_TYPES_H) || defined(HPP_GUARD_CTRLPP_TYPES_H) || defined(SPDLOG_VER_MAJOR)
    #error "the manipulator umbrella pulled in a cartan, ctrlpp or spdlog header"
#endif
#if __has_include(<cartan/lie.h>) || __has_include(<ctrlpp/trajectory.h>)
    #error "a cartan or ctrlpp include directory reached a translation unit linking only the manipulator module"
#endif

#include <array>
#include <memory>
#include <cstddef>

namespace {

// Every contract the module publishes, named in a translation unit that includes the umbrella and
// nothing else: a contract that stopped being reachable from the umbrella fails to compile here.
struct reachable_types
{
    praxis::manipulator::forward_kinematics_ops fk;
    praxis::manipulator::differential_kinematics_ops dk;
    praxis::manipulator::inverse_kinematics_ops ik;
    praxis::manipulator::robot_ops robot;
    praxis::manipulator::motion_ops motion;
    praxis::manipulator::modeling_ops modeling;
    praxis::manipulator::task_trajectory_ops trajectory;
    praxis::manipulator::capabilities composed;
    praxis::manipulator::solver_parameters parameters;
    praxis::manipulator::iteration_state state;
    praxis::manipulator::screw_chain chain;
    praxis::manipulator::jacobian derivative;
    praxis::trajectory::trajectory_sample sampled;
    praxis::manipulator::modeling_slot named;
    praxis::manipulator::modeling_slot_set expected;
    praxis::manipulator::forward_kinematics_slot forward_named;
    praxis::manipulator::differential_kinematics_slot differential_named;
    praxis::manipulator::inverse_kinematics_slot inverse_named;
};

// The five resolutions the module routes through inverse kinematics, each opened where it is called:
// a slot that stopped carrying the refusal channel fails to compile here.
std::size_t resolved(const praxis::manipulator::robot_ops &robot, const praxis::manipulator::motion_ops &motion, const praxis::manipulator::kinematics &solver)
{
    using praxis::rotation;
    using praxis::transform;
    using praxis::manipulator::joint_vector;

    const joint_vector seed;
    const Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    Eigen::Index reached       = robot.ik_solve_pose(solver, praxis::rigid_motion::frame_ops{}, transform::Identity(), seed, transform::Identity()).value_or(seed).size();
    reached += robot.ik_solve_flange_pose(solver, transform::Identity(), seed).value_or(seed).size();
    reached += motion.task_space_pose(solver, transform::Identity(), seed).value_or(seed).size();
    reached += motion.task_space_screw(praxis::rigid_motion::screw_ops{}, solver, transform::Identity(), axis, axis, 0.0, 0.0, seed).value_or(seed).size();
    reached += motion.tool_frame_displace(solver, transform::Identity(), axis, rotation::Identity(), seed).value_or(seed).size();

    return static_cast<std::size_t>(reached);
}

}

namespace praxis::manipulator::probe {

std::size_t reachable_surface()
{
    reachable_types types{};
    types.composed          = baseline();
    const kinematics solver = kinematics::compose(types.chain, types.fk, types.dk, types.ik, rigid_motion::screw_ops{}, rigid_motion::frame_ops{}).value_or(kinematics());
    auto motion             = types.trajectory.task_space_waypoints(solver, {}, to_joint_vector({}), joint_limits{});
    types.named             = modeling_slot::build_chain;
    types.derivative        = solver.space_jacobian(joint_vector()).value_or(jacobian());

    types.expected.set(types.named);
    types.sampled = motion ? (*motion)->sample((*motion)->duration()).value_or(trajectory::trajectory_sample{}) : trajectory::trajectory_sample{};

    const std::array<capability_view, 7> reported = capability_views(types.composed);

    return count_defaults(reported) + defaulted_slots(reported).size() + solver.iterations().size() + (motion ? 1u : 0u) + resolved(types.robot, types.motion, solver) +
            static_cast<std::size_t>(types.sampled.position.size()) + static_cast<std::size_t>(types.derivative.cols()) + (types.expected.contains(types.named) ? 1u : 0u) +
            (slot_name(reported.front(), 0).empty() ? 0u : 1u) + count_defaults(std::array<capability_view, 1>{view_of(types.modeling)});
}

}
