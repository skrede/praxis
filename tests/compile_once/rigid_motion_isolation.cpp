#include "praxis/rigid_motion.h"

#if defined(HPP_GUARD_CARTAN_TYPES_H) || defined(HPP_GUARD_CTRLPP_TYPES_H) || defined(HPP_GUARD_MEIOS_URDF_LOAD_H) || defined(SPDLOG_VER_MAJOR)
    #error "the rigid-motion umbrella pulled in a cartan, ctrlpp, meios or spdlog header"
#endif
// Logging is covered by the guard-macro form alone: spdlog is commonly installed as a system
// package, so its availability reports the machine rather than this target's link line.
#if __has_include(<cartan/lie.h>) || __has_include(<ctrlpp/trajectory.h>) || __has_include(<meios/urdf/load.h>)
    #error "a cartan, ctrlpp or meios include directory reached a translation unit linking only the rigid-motion module"
#endif

#include <array>
#include <memory>
#include <vector>
#include <cstddef>
#include <optional>

namespace {

// Every contract the module publishes, named in a translation unit that includes this extension's
// headers and nothing else: a contract that stopped being reachable fails to compile here.
struct reachable_types
{
    praxis::rigid_motion::frame_ops frame;
    praxis::rigid_motion::screw_ops screw;
    praxis::capability_view described;
    praxis::rigid_motion::capabilities composed;
    praxis::rigid_motion::frame_slot_set expected_frames;
    praxis::rigid_motion::screw_slot_set expected_screws;
    praxis::rigid_motion::frame_slot named_frame;
    praxis::rigid_motion::screw_slot named_screw;
};

}

namespace praxis::rigid_motion::probe {

std::size_t rigid_motion_surface()
{
    reachable_types types{};
    types.composed    = baseline();
    types.named_frame = frame_slot::rotate_z;
    types.named_screw = screw_slot::adjoint_map;

    types.expected_frames.set(types.named_frame);
    types.expected_screws.set(types.named_screw);
    types.described = view_of(types.screw);

    const std::array<capability_view, 2> reported{view_of(types.frame), types.described};

    return count_defaults(reported) + defaulted_slots(reported).size() + static_cast<std::size_t>(types.frame.rotate_z(0.0).size()) +
            (types.expected_frames.contains(types.named_frame) ? 1u : 0u) + (types.expected_screws.contains(types.named_screw) ? 1u : 0u) +
            (holds_default(types.described, static_cast<std::size_t>(types.named_screw)) ? 1u : 0u) + (slot_name(types.described, 0).empty() ? 0u : 1u) +
            count_defaults(capability_views(types.composed));
}

// The scenery this extension publishes, named the same way: a stencil, the window that drives it and
// the frame tree the two are arranged against.
std::size_t rigid_motion_scenery(threepp::Scene &target)
{
    frame_stencil body(target, std::vector<stencil_object>{stencil_object{"probe", axes_settings{}, object_body{}}}, frame_ops{});
    frame_window controls("probe", body, frame_ops{});
    frame_tree arranged_frames(1, frame_ops{});

    body.set_pose(0, transform::Identity());
    body.set_body(0, object_body{body_shape::none, 0.0, nullptr});
    body.set_axes_shown(0, true);
    arranged_frames.set_pose(0, body.world_pose(0));

    const frame_window::settings arranged = controls.state();
    const frame_window::placement driven  = arranged.objects.front();

    return static_cast<std::size_t>(body.pose(0).size() + driven.position.size() + driven.euler_degrees.size() + arranged_frames.pose(0).size() + arranged_frames.world_pose(0).size()) +
            controls.display_name().size() + body.count() + arranged_frames.count() + body.name_of(0).size() + arranged.objects.size() + (driven.order == axis_order::zyx ? 1u : 0u) +
            (driven.parent ? 1u : 0u) + (body.parent_of(0) ? 1u : 0u) + (body.axes_shown(0) ? 1u : 0u) + (arranged_frames.parent_of(0) ? 1u : 0u) +
            (body.set_parent(0, std::nullopt) ? 1u : 0u) + (arranged_frames.set_parent(0, std::nullopt) ? 1u : 0u);
}

}
