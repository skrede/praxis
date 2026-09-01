#include "scratch_directory.h"

#include "praxis/presets/arrangements.h"
#include "praxis/presets/arrangement_scenarios.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/rigid_motion/baseline/frame.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"
#include "praxis/config/document.h"

#include "praxis/scheduler/strand.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>

using namespace praxis;

namespace {

// How many times the substituted slot answered. A scenario composed against the baseline leaves it
// at zero, which is how a case tells the capabilities it passed from the ones the no-capability
// overload binds itself. The count rather than the value, because the composition asks this slot
// for the resting pose and would ask the reference for the same one.
int answered = 0;

transform counting_pose(const Eigen::Vector3d &p)
{
    ++answered;

    return rigid_motion::transformation_matrix_from_position(p);
}

rigid_motion::capabilities recording_capabilities()
{
    rigid_motion::capabilities motions                = rigid_motion::baseline();
    motions.frame.transformation_matrix_from_position = &counting_pose;

    return motions;
}

// A directory of this file's own, under the root the run removes as a tree when it ends.
std::filesystem::path scratch()
{
    const std::filesystem::path directory = fixture::shared_scratch_directory() / "arrangement-capability-binding";
    std::filesystem::create_directories(directory);

    return directory;
}

// The screw scenario reads none of the document's values, so what a case hands it only has to be a
// document: this one is the arrangement keyspace's own fallbacks, over a file that is not there.
config::document nothing_carried()
{
    const std::filesystem::path where = scratch() / "absent.xml";
    std::filesystem::remove(where);

    return config::load_or_defaults(presets::arrangement_keyspace(), config::resolve(where, scratch()), config::expectation::partial).values;
}

// The scene is the caller's, because the composition holds it and must not outlive it.
scene::preset_site headless(threepp::Scene &target)
{
    const scene::window_route nowhere = [](const std::shared_ptr<scene::imgui_window> &) {};

    return scene::preset_site{target, scheduler::strand{}, scheduler::strand{}, [] {}, nowhere, nowhere, {}};
}

// The screw scenario names its axis while it composes, so the slot answers before the preset is
// handed back and a case needs nothing running to read it.
constexpr std::size_t screw_scenario = 3u;

}

TEST_CASE("the_capabilities_a_composer_is_asked_for_are_what_the_scenario_composes_against")
{
    answered = 0;
    threepp::Scene target;

    const presets::arrangement_composer compose = presets::composer_for(presets::arrangement_scenario::screw, recording_capabilities());
    const std::shared_ptr<scene::preset> built  = compose(headless(target), nothing_carried());

    REQUIRE(built != nullptr);
    REQUIRE(answered > 0);
}

TEST_CASE("the_composer_asked_for_no_capabilities_binds_the_baseline_and_not_a_caller_s")
{
    answered = 0;
    threepp::Scene target;

    const presets::arrangement_composer compose = presets::composer_for(presets::arrangement_scenario::screw);
    const std::shared_ptr<scene::preset> built  = compose(headless(target), nothing_carried());

    REQUIRE(built != nullptr);
    CHECK(answered == 0);
}

TEST_CASE("register_arrangements_composes_a_registered_preset_against_the_capabilities_it_was_given")
{
    answered                              = 0;
    const std::filesystem::path directory = scratch();

    const std::string named{presets::arrangement_scenario_labels()[screw_scenario]};
    std::ofstream out(directory / "screw.xml", std::ios::binary | std::ios::trunc);
    out << "<arrangement><preset name=\"Screw motion: parameters\" scenario=\"" << named << "\"/></arrangement>\n";
    out.close();

    const std::vector<config::location> documents{config::resolve("screw.xml", directory)};
    const auto registry = std::make_shared<scene::preset_registry>();

    const std::vector<std::string> registered = presets::register_arrangements(registry, documents, {}, {}, recording_capabilities());
    REQUIRE(registered.size() == 1u);

    threepp::Scene target;
    const scene::preset_registry::factory compose = registry->load_preset(registered.front());
    REQUIRE(compose != nullptr);

    const std::shared_ptr<scene::preset> built = compose(headless(target));

    REQUIRE(built != nullptr);
    REQUIRE(answered > 0);
}
