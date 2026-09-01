#include "praxis/trajectory.h"

#if defined(HPP_GUARD_CARTAN_TYPES_H) || defined(HPP_GUARD_CTRLPP_TYPES_H) || defined(HPP_GUARD_MEIOS_URDF_LOAD_H) || defined(SPDLOG_VER_MAJOR)
    #error "the trajectory umbrella pulled in a cartan, ctrlpp, meios or spdlog header"
#endif
// Logging is covered by the guard-macro form alone: spdlog is commonly installed as a system
// package, so its availability reports the machine rather than this target's link line.
#if __has_include(<cartan/lie.h>) || __has_include(<ctrlpp/trajectory.h>) || __has_include(<meios/urdf/load.h>)
    #error "a cartan, ctrlpp or meios include directory reached a translation unit linking only the trajectory module"
#endif

#include <array>
#include <memory>
#include <cstddef>

namespace {

// Every contract the module publishes, named in a translation unit that includes this extension's
// headers and nothing else: a contract that stopped being reachable fails to compile here.
struct reachable_types
{
    praxis::trajectory::path_ops path;
    praxis::trajectory::trajectory_ops via_points;
    praxis::trajectory::time_scaling_ops time_scaling;
    praxis::trajectory::pose_trajectory_ops pose_trajectory;
    praxis::trajectory::scaling_sample scaled;
    praxis::trajectory::trajectory_sample sampled;
    praxis::trajectory::pose_sample posed;
    praxis::trajectory::configuration_limits bounds;
    praxis::capability_view described;
    praxis::trajectory::capabilities composed;
    praxis::trajectory::path_slot_set expected;
    praxis::trajectory::path_slot named;
};

}

namespace praxis::trajectory::probe {

std::size_t trajectory_surface()
{
    reachable_types types{};
    expected<std::unique_ptr<trajectory_generator>, refusal> motion    = types.via_points.joint_space_waypoints({}, configuration(), types.bounds);
    expected<std::unique_ptr<pose_trajectory_generator>, refusal> pose = types.pose_trajectory.decoupled_pose_waypoints({}, transform::Identity(), 1.0, 1.0);
    types.composed                                                     = baseline();
    types.named                                                        = path_slot::decoupled;

    types.expected.set(types.named);
    types.described = view_of(types.path);
    types.scaled    = types.time_scaling.quintic(0.0, 1.0).value_or(scaling_sample{});
    if(!motion || !pose)
        return 0;

    types.sampled = (*motion)->sample(types.scaled.s + types.scaled.ds + types.scaled.dds).value_or(trajectory_sample{});
    types.posed   = (*pose)->sample((*pose)->duration() + (*motion)->duration()).value_or(pose_sample{});

    const std::array<capability_view, 4> reported{types.described, view_of(types.via_points), view_of(types.time_scaling), view_of(types.pose_trajectory)};

    return count_defaults(reported) + defaulted_slots(reported).size() + static_cast<std::size_t>(types.sampled.position.size() + types.sampled.velocity.size()) +
            static_cast<std::size_t>(types.sampled.acceleration.size() + types.posed.velocity.size() + types.posed.position.size()) +
            static_cast<std::size_t>(types.path.screw(types.posed.position, transform::Identity(), 0.0).value_or(transform::Identity()).size()) +
            (types.expected.contains(types.named) ? 1u : 0u) + (holds_default(types.described, static_cast<std::size_t>(types.named)) ? 1u : 0u) +
            (slot_name(types.described, 0).empty() ? 0u : 1u) + count_defaults(capability_views(types.composed));
}

}
