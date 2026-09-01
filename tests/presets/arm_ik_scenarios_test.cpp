#include "described_arm.h"
#include "composed_panels.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/slots.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/config/configurable.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <set>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <string_view>

using namespace praxis;
using namespace praxis::fixture;

namespace {

const std::vector<std::string> &numerical_windows()
{
    static const std::vector<std::string> shown{"Joint control", "Target pose", "Starts", "Solutions", "Iterations", "Convergence", "View"};

    return shown;
}

const std::vector<std::string> &analytic_windows()
{
    static const std::vector<std::string> shown{"Joint control", "Target pose", "Solutions", "View"};

    return shown;
}

// Every capability the reference implementation binds, with whichever of them a case names left at
// its inert default.
manipulator::capabilities bound_except(bool solves)
{
    manipulator::capabilities arm = manipulator::baseline();
    if(!solves)
        arm.ik = manipulator::inverse_kinematics_ops{};

    return arm;
}

std::shared_ptr<scene::preset> composed_arm(const scene::preset_site &site, const presets::arm_scenario &chosen, const manipulator::arm_composition &opened, bool solves = true)
{
    return presets::arm_preset(site, bound_except(solves), trajectory::baseline(), rigid_motion::baseline(), chosen, opened);
}

// Where each window of one composition keeps its settings, with the windows keeping none left out. A
// path two windows answered would make one of them write over the other's values.
std::vector<std::string_view> settings_paths(const std::shared_ptr<scene::preset> &composed)
{
    REQUIRE(composed != nullptr);

    std::vector<std::string_view> named;
    for(const std::shared_ptr<scene::imgui_window> &panel : composed->windows)
        if(const config::configurable *carried = panel->as_configurable(); carried != nullptr)
            named.push_back(carried->settings_path());

    return named;
}

}

TEST_CASE("the numerical scenario composes the seven windows its own solve needs", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    REQUIRE(composed_windows(composed_arm(unwired(target), chosen, presets::arm_windows_numerical_ik(chosen))) == numerical_windows());
}

TEST_CASE("the analytic scenario opens neither the starts, the steps nor the curve", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    REQUIRE(composed_windows(composed_arm(unwired(target), chosen, presets::arm_windows_analytic_ik(chosen))) == analytic_windows());
}

// An unbound slot is said out loud where a solve is asked for, so the scene a learner who has bound
// neither of them opens is the scene they will have once they bind one.
TEST_CASE("both scenarios open their windows over an arm nothing can solve for", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene searched;
    REQUIRE(composed_windows(composed_arm(unwired(searched), chosen, presets::arm_windows_numerical_ik(chosen), false)) == numerical_windows());

    threepp::Scene closed;
    REQUIRE(composed_windows(composed_arm(unwired(closed), chosen, presets::arm_windows_analytic_ik(chosen), false)) == analytic_windows());
}

TEST_CASE("every window the two scenarios compose opens exactly one panel under its own title", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene searched;
    each_window_opens_one_panel(composed_arm(unwired(searched), chosen, presets::arm_windows_numerical_ik(chosen)));

    threepp::Scene closed;
    each_window_opens_one_panel(composed_arm(unwired(closed), chosen, presets::arm_windows_analytic_ik(chosen)));
}

TEST_CASE("no two windows of one scenario keep their settings under the same key path", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene searched;
    const std::vector<std::string_view> numerical = settings_paths(composed_arm(unwired(searched), chosen, presets::arm_windows_numerical_ik(chosen)));
    REQUIRE(numerical.size() == numerical_windows().size());
    REQUIRE(std::set<std::string_view>(numerical.begin(), numerical.end()).size() == numerical.size());

    threepp::Scene closed;
    const std::vector<std::string_view> analytic = settings_paths(composed_arm(unwired(closed), chosen, presets::arm_windows_analytic_ik(chosen)));
    REQUIRE(analytic.size() == analytic_windows().size());
    REQUIRE(std::set<std::string_view>(analytic.begin(), analytic.end()).size() == analytic.size());
}

// A path the struct names and the table does not is a path nothing walks, and a path listed twice is
// one whose second entry proves nothing.
TEST_CASE("the table of window key paths carries every path once", "[presets][windows]")
{
    const std::set<std::string_view> listed(presets::window_paths::every.begin(), presets::window_paths::every.end());

    REQUIRE(listed.size() == presets::window_paths::counted);
    REQUIRE(listed.contains(presets::window_paths::ik_seeds));
    REQUIRE(listed.contains(presets::window_paths::ik_branch));
    REQUIRE(listed.contains(presets::window_paths::ik_iterates));
    REQUIRE(listed.contains(presets::window_paths::ik_convergence));
}
