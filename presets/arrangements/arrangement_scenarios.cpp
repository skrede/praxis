#include "praxis/presets/screw.h"
#include "praxis/presets/two_pose.h"
#include "praxis/presets/twist_axis.h"
#include "praxis/presets/euler_rungs.h"
#include "praxis/presets/arrangements.h"
#include "praxis/presets/rotation_axis.h"
#include "praxis/presets/frame_workbench.h"
#include "praxis/presets/arrangement_scenarios.h"

#include "praxis/rigid_motion/capabilities.h"

#include <span>
#include <array>
#include <cstddef>

namespace praxis::presets {

namespace {

// In the enumeration's own order, which is what indexing the composer table below by an enumerator
// relies on.
constexpr std::array<const char *, 7> scenario_spellings{"euler pose", "euler relative", "frame workbench", "screw", "twist axis", "two pose", "rotation axis"};

// The composer for one scenario, over the spatial capability every one of the seven is built on.
using composer_factory = arrangement_composer (*)(const rigid_motion::capabilities &);

arrangement_composer euler_pose_composer(const rigid_motion::capabilities &motions)
{
    return [motions](const scene::preset_site &site, const config::document &carried)
    { return euler_rung_preset(site, motions, euler_rung::single_frame, arrangement_source{arrangement_path, carried}); };
}

arrangement_composer euler_relative_composer(const rigid_motion::capabilities &motions)
{
    return [motions](const scene::preset_site &site, const config::document &carried)
    { return euler_rung_preset(site, motions, euler_rung::paired_frames, arrangement_source{arrangement_path, carried}); };
}

arrangement_composer frame_workbench_composer(const rigid_motion::capabilities &motions)
{
    return [motions](const scene::preset_site &site, const config::document &carried) { return frame_workbench_preset(site, motions, arrangement_source{arrangement_path, carried}); };
}

// The four below take no arrangement source and read none of the document's values, so what they
// draw is the arrangement each of them supplies itself whatever the document carries.
arrangement_composer screw_composer(const rigid_motion::capabilities &motions)
{
    return [motions](const scene::preset_site &site, const config::document &) { return screw_preset(site, motions); };
}

arrangement_composer twist_axis_composer(const rigid_motion::capabilities &motions)
{
    return [motions](const scene::preset_site &site, const config::document &) { return twist_axis_preset(site, motions); };
}

arrangement_composer two_pose_composer(const rigid_motion::capabilities &motions)
{
    return [motions](const scene::preset_site &site, const config::document &) { return two_pose_preset(site, motions); };
}

arrangement_composer rotation_axis_composer(const rigid_motion::capabilities &motions)
{
    return [motions](const scene::preset_site &site, const config::document &) { return rotation_axis_preset(site, motions); };
}

constexpr std::array<composer_factory, 7> offered_scenarios{&euler_pose_composer, &euler_relative_composer, &frame_workbench_composer, &screw_composer,
                                                            &twist_axis_composer, &two_pose_composer,       &rotation_axis_composer};

}

std::span<const char *const> arrangement_scenario_labels()
{
    return scenario_spellings;
}

arrangement_composer composer_for(arrangement_scenario scenario, const rigid_motion::capabilities &motions)
{
    return offered_scenarios[static_cast<std::size_t>(scenario)](motions);
}

arrangement_composer composer_for(arrangement_scenario scenario)
{
    return composer_for(scenario, rigid_motion::baseline());
}

}
