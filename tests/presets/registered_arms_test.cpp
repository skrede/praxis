#include "described_arm.h"
#include "scratch_directory.h"

#include "praxis/presets/arm_scenarios.h"
#include "praxis/presets/arm_registration.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/scheduler/strand.h"

#include "praxis/config/store.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <filesystem>

using namespace praxis;

namespace {

// A directory of its own for each case, under a root this run holds and removes as a tree when it
// ends, so a document one case authors is invisible to the next.
std::filesystem::path scratch(const char *named)
{
    const std::filesystem::path directory = fixture::shared_scratch_directory() / named;
    std::filesystem::create_directories(directory);

    return directory;
}

// A document is written out and then named, because what is under test is a preset existing because
// its document does.
config::location written(const std::filesystem::path &directory, const std::string &file, const std::string &body)
{
    std::ofstream out(directory / file, std::ios::binary | std::ios::trunc);
    out << body << "\n";
    out.close();

    return config::resolve(file, directory);
}

std::string spelling(presets::arm_scenario_kind scenario)
{
    return presets::arm_scenario_labels()[static_cast<std::size_t>(scenario)];
}

std::string starting_at(std::size_t joints)
{
    std::string listed;
    for(std::size_t axis = 0u; axis < joints; ++axis)
        listed += "<joint index=\"" + std::to_string(axis) + "\" degrees=\"0\"/>";

    return listed;
}

config::location authored(const std::filesystem::path &directory, const std::string &file, const std::string &name, const std::string &description, std::size_t joints)
{
    return written(directory, file,
                   "<arm><preset name=\"" + name + "\" scenario=\"" + spelling(presets::arm_scenario_kind::forward_kinematics) + "\"/><description path=\"" + description +
                           "\"/><initial>" + starting_at(joints) + "</initial></arm>");
}

// The scene is the caller's, because the composition holds it and must not outlive it. A name the
// registry does not carry answers an empty factory, so it is refused here rather than called.
std::shared_ptr<scene::preset> composed(scene::preset_registry &registry, const std::string &name, threepp::Scene &target)
{
    const scene::window_route nowhere             = [](const std::shared_ptr<scene::imgui_window> &) {};
    const scene::preset_registry::factory compose = registry.load_preset(name);
    REQUIRE(compose != nullptr);

    return compose(scene::preset_site{target, scheduler::strand{}, scheduler::strand{}, [] {}, nowhere, nowhere, {}});
}

std::vector<std::string> opened_through(const std::vector<config::location> &documents, const std::vector<std::filesystem::path> &roots)
{
    const auto registry                       = std::make_shared<scene::preset_registry>();
    const std::vector<std::string> registered = presets::register_arms(registry, documents, roots, {}, {});
    REQUIRE(registered.size() == 1u);

    threepp::Scene target;
    const std::shared_ptr<scene::preset> built = composed(*registry, registered.front(), target);
    REQUIRE(built != nullptr);

    std::vector<std::string> named;
    for(const std::shared_ptr<scene::imgui_window> &panel : built->windows)
        named.push_back(panel->display_name());

    return named;
}

}

TEST_CASE("an arm composes from a document naming a description a supplied root holds", "[presets][registry]")
{
    const fixture::described_arm described(6, "six");
    const std::filesystem::path directory     = scratch("described_arm");
    const std::vector<config::location> named = {authored(directory, "six.xml", "Six axes", "six.urdf", 6)};
    const std::vector<std::filesystem::path> roots{described.directory};

    const auto registry = std::make_shared<scene::preset_registry>();
    REQUIRE(presets::register_arms(registry, named, roots, {}, {}) == std::vector<std::string>{"Six axes"});

    threepp::Scene target;
    REQUIRE(composed(*registry, "Six axes", target) != nullptr);
}

TEST_CASE("the names answered are the ones the span carried, in the order it carried them", "[presets][registry]")
{
    const fixture::described_arm described(6, "six");
    const std::filesystem::path directory = scratch("arm_ordering");
    const std::vector<config::location> named{authored(directory, "second.xml", "Second", "six.urdf", 6), authored(directory, "first.xml", "First", "six.urdf", 6)};
    const std::vector<std::filesystem::path> roots{described.directory};

    const auto registry = std::make_shared<scene::preset_registry>();
    REQUIRE(presets::register_arms(registry, named, roots, {}, {}) == std::vector<std::string>{"Second", "First"});
}

TEST_CASE("a span carrying no documents registers nothing", "[presets][registry]")
{
    const auto registry = std::make_shared<scene::preset_registry>();

    REQUIRE(presets::register_arms(registry, {}, {}, {}, {}).empty());
    REQUIRE(registry->preset_names().empty());
}

TEST_CASE("a description no supplied root holds still registers its preset", "[presets][registry]")
{
    const std::filesystem::path directory = scratch("arm_unresolved");
    const std::vector<config::location> named{authored(directory, "six.xml", "Six axes", "elsewhere/six.urdf", 6)};

    const auto registry = std::make_shared<scene::preset_registry>();
    REQUIRE(presets::register_arms(registry, named, {}, {}, {}) == std::vector<std::string>{"Six axes"});

    threepp::Scene target;
    REQUIRE(composed(*registry, "Six axes", target) == nullptr);
}

TEST_CASE("a description resolves the same against one root and against two whose first holds nothing", "[presets][registry]")
{
    const fixture::described_arm described(6, "six");
    const std::filesystem::path directory = scratch("arm_roots");
    const std::filesystem::path barren    = scratch("arm_roots_barren");
    const std::vector<config::location> named{authored(directory, "six.xml", "Six axes", "six.urdf", 6)};

    REQUIRE(opened_through(named, {described.directory}) == opened_through(named, {barren, described.directory}));
}
