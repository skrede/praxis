#include "demo_offered.h"
#include "demo_configuration.h"

#include "praxis/presets/arm_registration.h"

#include "praxis/manipulator/capabilities.h"

#include "praxis/scene/coverage_report.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/config/store.h"

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>

namespace praxis::demo {

namespace {

// What this scenario composes and reports on. It is the demonstration's own glue: the library
// publishes the capabilities, and which of them a scenario binds is the scenario's choice.
struct composed_capabilities
{
    manipulator::capabilities arm;
    trajectory::capabilities shapes;
    rigid_motion::capabilities motions;
};

// The report is over what this scenario composed and nothing else, so its width is the number of
// capabilities above rather than anything the library fixes.
std::array<capability_view, 13> composed_views(const composed_capabilities &composed)
{
    return {view_of(composed.arm.fk),
            view_of(composed.arm.dk),
            view_of(composed.arm.ik),
            view_of(composed.arm.robot),
            view_of(composed.arm.motion),
            view_of(composed.arm.modeling),
            view_of(composed.arm.trajectory),
            view_of(composed.shapes.time_scaling),
            view_of(composed.shapes.path),
            view_of(composed.shapes.pose_trajectory),
            view_of(composed.shapes.trajectory),
            view_of(composed.motions.frame),
            view_of(composed.motions.screw)};
}

// The views point into the value passed, so a temporary would leave every one of them dangling at the
// end of the full expression.
std::array<capability_view, 13> composed_views(composed_capabilities &&) = delete;

composed_capabilities bound_capabilities()
{
    return composed_capabilities{manipulator::baseline(), trajectory::baseline(), rigid_motion::baseline()};
}

void report_composed_capabilities()
{
    const composed_capabilities composed = bound_capabilities();

    scene::report_default_slots(composed_views(composed));
}

}

std::vector<std::string> register_offered(const std::shared_ptr<scene::preset_registry> &registry, const documents &mine, const config::document &values,
                                          const std::filesystem::path &packages, const std::shared_ptr<write_back> &writing)
{
    report_composed_capabilities();

    const std::vector<config::location> reading = preset_locations(values, mine);
    const std::array<std::filesystem::path, 1> roots{packages};

    return presets::register_arms(
            registry, reading, roots, [mine](const std::filesystem::path &named) { return mine.composing(named); },
            [writing](const config::binding &at, const config::document &carried) { writing->composing(at, carried); });
}

}
