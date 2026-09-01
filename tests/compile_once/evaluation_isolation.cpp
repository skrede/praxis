#include "praxis/evaluation.h"

#if defined(HPP_GUARD_CARTAN_LIE_H) || defined(HPP_GUARD_CTRLPP_CONTROL_H) || defined(HPP_GUARD_MEIOS_URDF_LOAD_H) || defined(SPDLOG_VER_MAJOR)
    #error "the evaluation umbrella pulled in a solver, control, model-library or logging header"
#endif
#if defined(HPP_GUARD_PRAXIS_MANIPULATOR_H) || defined(HPP_GUARD_PRAXIS_TRAJECTORY_H) || defined(HPP_GUARD_PRAXIS_RIGID_MOTION_H) ||                                                    \
        defined(HPP_GUARD_PRAXIS_SCENE_PRESET_H)
    #error "the evaluation umbrella pulled in an extension's umbrella or a scene header"
#endif

// The guard-macro form above only fires once a header is actually included. A dependency whose
// include directories reach the target while none of its headers happens to be included is invisible
// to it, so the availability form below is what asserts the dependency footprint itself.
#if __has_include(<cartan/lie.h>) || __has_include(<ctrlpp/control.h>) || __has_include(<meios/urdf/load.h>)
    #error "a solver, control or model-library include directory reached a translation unit linking only the evaluation module"
#endif
// Logging is covered by the guard-macro form alone: spdlog is commonly installed as a system
// package, so its availability reports the machine rather than this target's link line.
#if __has_include(<threepp/threepp.hpp>) || __has_include(<imgui.h>)
    #error "a renderer or GUI include directory reached a translation unit linking only the evaluation module"
#endif
#if __has_include("praxis/manipulator.h") || __has_include("praxis/trajectory.h") || __has_include("praxis/rigid_motion.h")
    #error "an extension's umbrella reached a translation unit linking only the evaluation module"
#endif
#if __has_include("praxis/rigid_motion/types.h") || __has_include("praxis/scene/preset.h") || __has_include("praxis/scheduler/strand.h")
    #error "an extension's, the scene's or the scheduler's include directory reached a translation unit linking only the evaluation module"
#endif

#include <span>
#include <array>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace {

Eigen::Matrix3d unturned(double)
{
    return Eigen::Matrix3d::Identity();
}

// The facility describes no capability of its own, so the probe declares one: this is the whole
// surface a comparison authored outside this repository reaches.
struct probe_ops
{
    Eigen::Matrix3d (*turn)(double radians) = &unturned;
};

praxis::evaluation::case_result compared_turn(const void *first, const void *second, praxis::evaluation::case_source &drawn,
                                              const praxis::evaluation::tolerance_pair &allowed)
{
    const double radians = drawn.angle_radians();
    const praxis::evaluation::residual difference =
            praxis::evaluation::geodesic_residual(static_cast<const probe_ops *>(first)->turn(radians), static_cast<const probe_ops *>(second)->turn(radians));

    return praxis::evaluation::case_result{praxis::evaluation::verdict_of(difference, allowed), difference};
}

constexpr std::array probe_table{
        praxis::evaluation::slot_evaluation{"probe.turn", praxis::evaluation::residual_kind::geodesic,
                                            praxis::evaluation::tolerance_of(praxis::evaluation::residual_kind::geodesic), &compared_turn},
};

constexpr praxis::evaluation::capability_evaluations<probe_ops> described_probes{"probe", probe_table};

// Every contract the module publishes, named in a translation unit that includes this module's
// umbrella and nothing else: a contract that stopped being reachable fails to compile here.
struct reachable_types
{
    praxis::evaluation::residual difference;
    praxis::evaluation::residual_kind kind;
    praxis::evaluation::agreement verdict;
    praxis::evaluation::tolerance_pair allowed;
    praxis::evaluation::case_result answer;
    praxis::evaluation::slot_evaluation described;
    praxis::evaluation::evaluation_view compared;
    praxis::evaluation::slot_report reported;
};

}

namespace praxis::evaluation::probe {

std::size_t drawn_surface()
{
    case_source drawn        = case_source::for_slot(0x5EEDu, "probe.turn");
    const double radians     = drawn.angle_radians();
    const Eigen::Matrix3d at = unturned(radians);

    return static_cast<std::size_t>(drawn.seed() % 7u) + (is_approx_equal(radians, radians, default_tolerance) ? 1u : 0u) + (is_approx_equal(at, at) ? 1u : 0u);
}

std::size_t ledger_surface(std::span<const evaluation_view> compared)
{
    const std::array<capability_view, 0> described{};

    return unevaluated_slots(described, compared).size() + unnamed_evaluations(described, compared).size();
}

std::size_t evaluation_surface()
{
    const probe_ops first{};
    const probe_ops second{};
    const std::array<evaluation_view, 1> compared{evaluation_view::of(first, second, described_probes)};
    const evaluation_report whole = evaluate(compared, 0x5EEDu, 4);
    reachable_types types{};

    types.described  = probe_table.front();
    types.compared   = compared.front();
    types.reported   = whole.slots.front();
    types.difference = types.reported.worst;
    types.kind       = types.difference.kind;
    types.allowed    = types.described.allowed;
    types.verdict    = verdict_of(types.difference, tolerance_of(types.kind));
    types.answer     = case_result{types.verdict, types.difference};

    return (every_slot_agreed(whole) ? 1u : 0u) + disagreeing_slots(whole).size() + types.compared.extension().size() + types.compared.slots().size() + types.reported.cases +
            whole.cases_per_slot + (types.answer.verdict == agreement::agreed ? 1u : 0u) + drawn_surface() + ledger_surface(compared);
}

}
