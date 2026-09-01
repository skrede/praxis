#if __has_include("praxis/manipulator/screw_chain_builder.h")
    #include "praxis/manipulator/robot_controller.h"
    #include "praxis/manipulator/screw_chain_builder.h"
    #include "praxis/manipulator/baseline/kinematics.h"
#endif

#if __has_include("praxis/scene/visualizer.h")
    #include "praxis/scene/visualizer.h"
#endif

#if __has_include("praxis/scheduler/scheduler.h")
    #include "praxis/scheduler/scheduler.h"
#endif

#if __has_include("praxis/compat/detail/callable.h")
    #include "praxis/compat/detail/callable.h"
#endif

#if __has_include("praxis/rigid_motion/baseline/frame.h")
    #include "praxis/rigid_motion/baseline/frame.h"
#endif

#if __has_include("praxis/trajectory/path.h")
    #include "praxis/trajectory/path.h"
#endif

#if __has_include("praxis/trajectory/pose_trajectory.h")
    #include "praxis/trajectory/pose_trajectory.h"
#endif

#if __has_include("praxis/extension.h")
    #include "praxis/extension.h"
#endif

#include <span>
#include <array>
#include <memory>
#include <cstddef>
#include <cstdint>

// At least one symbol per namespaced target: configure fails if an alias is missing, the compile
// fails if a module's headers are out of reach or a return type it publishes has moved, and the link
// fails if a module does not build. The addresses sit in mutable objects with external linkage,
// which a translation unit must emit at every optimization level. None of them is virtual on purpose
// -- a pointer to a virtual member is an offset into a vtable and would reference no symbol the
// library defines. Each symbol is guarded on the header it names, so a module the umbrella's list no
// longer carries drops out of this file instead of breaking it.
#if __has_include("praxis/extension.h")
std::size_t (*praxis_extension_symbol)(std::span<const praxis::capability_view>) = &praxis::count_defaults;
#endif

#if __has_include("praxis/trajectory/path.h")
praxis::expected<praxis::transform, praxis::refusal> (*praxis_trajectory_path_symbol)(const praxis::transform &, const praxis::transform &, double) = &praxis::trajectory::inert::screw;
#endif

#if __has_include("praxis/trajectory/pose_trajectory.h")
praxis::expected<std::unique_ptr<praxis::trajectory::pose_trajectory_generator>, praxis::refusal> (*praxis_trajectory_inert_symbol)(
        std::span<const praxis::transform>, const praxis::transform &, double, double) = &praxis::trajectory::inert::decoupled_pose_waypoints;
#endif

#if __has_include("praxis/rigid_motion/baseline/frame.h")
praxis::rotation (*praxis_rigid_motion_symbol)(double) = &praxis::rigid_motion::rotate_z;
#endif

#if __has_include("praxis/compat/detail/callable.h")
void (*praxis_compat_symbol)() = &praxis::detail::called_without_target;
#endif

#if __has_include("praxis/scheduler/scheduler.h")
bool (praxis::scheduler::scheduler::*praxis_scheduler_symbol)() = &praxis::scheduler::scheduler::step;
#endif

#if __has_include("praxis/scene/visualizer.h")
bool (praxis::scene::visualizer::*praxis_scene_symbol)() = &praxis::scene::visualizer::render_once;
#endif

#if __has_include("praxis/manipulator/screw_chain_builder.h")
praxis::expected<praxis::manipulator::screw_chain, praxis::refusal> (*praxis_manipulator_model_symbol)(const meios::model<> &) = &praxis::manipulator::build_screw_chain;

praxis::expected<praxis::manipulator::kinematics, praxis::refusal> (*praxis_manipulator_reference_symbol)(
        const praxis::manipulator::screw_chain &, praxis::manipulator::forward_kinematics_ops, praxis::manipulator::differential_kinematics_ops,
        praxis::manipulator::inverse_kinematics_ops, const praxis::rigid_motion::screw_ops &, const praxis::rigid_motion::frame_ops &) = &praxis::manipulator::make_kinematics;

std::uint32_t (praxis::manipulator::robot_controller::*praxis_manipulator_control_symbol)() const = &praxis::manipulator::robot_controller::joint_count;
#endif

int main()
{
    return 0;
}
