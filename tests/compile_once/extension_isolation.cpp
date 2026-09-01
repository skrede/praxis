#include "praxis/extension.h"

#if defined(HPP_GUARD_CARTAN_LIE_H) || defined(HPP_GUARD_CTRLPP_CONTROL_H) || defined(HPP_GUARD_MEIOS_URDF_LOAD_H)
    #error "the extension machinery umbrella pulled in a solver, control or model-library header"
#endif
#if defined(THREEPP_OBJECT3D_HPP) || defined(IMGUI_VERSION) || defined(SPDLOG_VER_MAJOR)
    #error "the extension machinery umbrella pulled in a renderer, GUI or logging header"
#endif
// The scene publishes no umbrella, so it is named by a header in the second block alone; the two
// blocks together reach each of the four other modules of this repository exactly once.
#if defined(HPP_GUARD_PRAXIS_MANIPULATOR_H) || defined(HPP_GUARD_PRAXIS_TRAJECTORY_H) || defined(HPP_GUARD_PRAXIS_RIGID_MOTION_H)
    #error "the extension machinery umbrella pulled in another first-party module's umbrella"
#endif
#if defined(HPP_GUARD_PRAXIS_MANIPULATOR_TYPES_H) || defined(HPP_GUARD_PRAXIS_TRAJECTORY_TYPES_H) || defined(HPP_GUARD_PRAXIS_RIGID_MOTION_TYPES_H) ||                                  \
        defined(HPP_GUARD_PRAXIS_SCENE_PRESET_H)
    #error "the extension machinery umbrella pulled in another first-party module's header"
#endif

// The guard-macro form above only fires once a header is actually included. A dependency whose
// include directories reach the target while none of its headers happens to be included is invisible
// to it, so the availability form below is what asserts the dependency footprint itself.
#if __has_include(<cartan/lie.h>) || __has_include(<ctrlpp/control.h>) || __has_include(<meios/urdf/load.h>)
    #error "a solver, control or model-library include directory reached a translation unit linking only the extension machinery"
#endif
// Logging is covered by the guard-macro form alone: spdlog is commonly installed as a system
// package, so its availability reports the machine rather than this target's link line.
#if __has_include(<threepp/threepp.hpp>) || __has_include(<imgui.h>)
    #error "a renderer or GUI include directory reached a translation unit linking only the extension machinery"
#endif
#if __has_include("praxis/manipulator.h") || __has_include("praxis/trajectory.h") || __has_include("praxis/rigid_motion.h")
    #error "another first-party module's umbrella reached a translation unit linking only the extension machinery"
#endif
#if __has_include("praxis/manipulator/types.h") || __has_include("praxis/trajectory/types.h") || __has_include("praxis/rigid_motion/types.h") || __has_include("praxis/scene/preset.h")
    #error "another first-party module's include directory reached a translation unit linking only the extension machinery"
#endif

#include <array>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace {

double origin()
{
    return 0.0;
}

double unit()
{
    return 1.0;
}

// The machinery declares no capability of its own, so the probe declares one: this is the whole
// surface an extension authored outside this repository reaches.
struct probe_ops
{
    double (*offset)() = &origin;
    double (*scale)()  = &unit;
};

enum class probe_slot : std::uint32_t
{
    offset,
    scale,
    count,
};

constexpr std::array probe_descriptors{
        praxis::slot_descriptor{"probe.offset", [](const void *value) -> bool { return static_cast<const probe_ops *>(value)->offset == &origin; }},
        praxis::slot_descriptor{"probe.scale", [](const void *value) -> bool { return static_cast<const probe_ops *>(value)->scale == &unit; }},
};

static_assert(praxis::slot_enumeration<probe_slot>);
static_assert(probe_descriptors.size() == static_cast<std::size_t>(probe_slot::count));

constexpr praxis::capability_descriptors<probe_ops> described_probes{"probe", probe_descriptors};

// Every type the umbrella publishes, named in a translation unit that includes it and nothing else:
// a type that stopped being reachable from the umbrella fails to compile here.
struct reachable_types
{
    praxis::slot_descriptor descriptor;
    praxis::capability_view described;
    praxis::defaulted_slot entry;
    praxis::basic_slot_set<probe_slot> expected;
};

}

namespace praxis::probe {

std::size_t slot_surface()
{
    const probe_ops ops{};
    reachable_types types{};

    types.descriptor = probe_descriptors.front();
    types.described  = capability_view::of(ops, described_probes);
    types.expected.set(probe_slot::scale);

    const std::array<capability_view, 1> reported{types.described};
    const std::vector<defaulted_slot> held      = defaulted_slots(reported);
    const basic_slot_set<probe_slot> every_slot = types.expected | ~types.expected;
    const std::string_view named                = slot_name(types.described, 0);

    types.entry = held.front();

    return count_defaults(reported) + held.size() + types.entry.extension.size() + types.entry.slot.size() + (types.expected.contains(probe_slot::scale) ? 1u : 0u) +
            ((every_slot & types.expected).empty() ? 0u : 1u) + (holds_default(types.described, 0) ? 1u : 0u) + (named == types.descriptor.name ? 1u : 0u);
}

}
