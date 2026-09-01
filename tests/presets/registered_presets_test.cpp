#include "scratch_directory.h"

#include "praxis/presets/arrangements.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"

#include "praxis/scheduler/strand.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <algorithm>
#include <filesystem>

using namespace praxis;

namespace {

// One document a case authors: its file, the name it states, and its scenario's place in the list.
struct authored_preset
{
    const char *file;
    const char *name;
    std::size_t scenario;
};

// The seven documents the demonstration ships, under exactly the names its selector shows.
constexpr std::array<authored_preset, 7> shipped{{{"euler-pose.xml", "Reference frame: pose", 0u},
                                                  {"euler-relative.xml", "Reference frames: perspectives", 1u},
                                                  {"frame-workbench.xml", "Reference frames", 2u},
                                                  {"screw.xml", "Screw motion: parameters", 3u},
                                                  {"twist-axis.xml", "Screw motion: twist interpolation", 4u},
                                                  {"two-pose.xml", "Screw motion: pose interpolation", 5u},
                                                  {"rotation-axis.xml", "Rotation: exponential coordinates", 6u}}};

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

config::location authored(const std::filesystem::path &directory, const std::string &file, const std::string &name, const std::string &scenario)
{
    return written(directory, file, "<arrangement><preset name=\"" + name + "\" scenario=\"" + scenario + "\"/></arrangement>");
}

// The spellings come from the published list, so a case cannot go on covering one it stopped admitting.
std::vector<config::location> every(const std::filesystem::path &directory)
{
    std::vector<config::location> named;
    for(const authored_preset &one : shipped)
        named.push_back(authored(directory, one.file, one.name, presets::arrangement_scenario_labels()[one.scenario]));

    return named;
}

std::vector<std::string> offered()
{
    std::vector<std::string> named;
    for(const authored_preset &one : shipped)
        named.emplace_back(one.name);

    return named;
}

// Sorted, because `preset_names` reads an unordered map and the order it answers in is unspecified:
// a case comparing that against a sequence would assert something the registry does not promise.
std::vector<std::string> sorted(std::vector<std::string> named)
{
    std::sort(named.begin(), named.end());

    return named;
}

// A document offering no preset leaves the registry as it found it and contributes no name.
void registers_nothing(const std::vector<config::location> &documents)
{
    const auto registry = std::make_shared<scene::preset_registry>();

    REQUIRE(presets::register_arrangements(registry, documents, {}, {}).empty());
    REQUIRE(registry->preset_names().empty());
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

// Each of them composed once, over a scene of its own.
void compose_each(scene::preset_registry &registry)
{
    for(const std::string &name : offered())
    {
        INFO(name);
        threepp::Scene target;
        const std::shared_ptr<scene::preset> built = composed(registry, name, target);

        REQUIRE(built != nullptr);
        REQUIRE_FALSE(built->windows.empty());
    }
}

// The document each composition announced it was built from, in the order the compositions happened.
presets::composed_route recording(std::vector<std::filesystem::path> &built_from)
{
    return [&built_from](const config::binding &at, const config::document &) { built_from.push_back(at.at.resolved); };
}

// One document, under the first of the names and the first of the scenarios the set carries.
config::location one(const std::filesystem::path &directory, const std::string &file)
{
    return authored(directory, file, shipped.front().name, presets::arrangement_scenario_labels()[shipped.front().scenario]);
}

// One directory answered over another, and the other only while no document of that name stands in
// the first.
presets::document_route preferring(const std::filesystem::path &preferred, const std::filesystem::path &otherwise)
{
    return [preferred, otherwise](const std::filesystem::path &named)
    {
        const config::location standing = config::resolve(named, preferred);

        return std::filesystem::exists(standing.resolved) ? standing : config::resolve(named, otherwise);
    };
}

}

TEST_CASE("the arrangements register under exactly the names their documents state", "[presets][registry]")
{
    const std::filesystem::path directory = scratch("every");
    const auto registry                   = std::make_shared<scene::preset_registry>();

    REQUIRE(presets::register_arrangements(registry, every(directory), {}, {}) == offered());
    REQUIRE(sorted(registry->preset_names()) == sorted(offered()));
}

TEST_CASE("a directory holding no document registers nothing", "[presets][registry]")
{
    registers_nothing({});
}

TEST_CASE("registering the same documents twice does not multiply the names", "[presets][registry]")
{
    const std::filesystem::path directory         = scratch("twice");
    const auto registry                           = std::make_shared<scene::preset_registry>();
    const std::vector<config::location> documents = every(directory);

    REQUIRE(presets::register_arrangements(registry, documents, {}, {}) == offered());
    REQUIRE(presets::register_arrangements(registry, documents, {}, {}).empty());
    REQUIRE(sorted(registry->preset_names()) == sorted(offered()));
}

TEST_CASE("each registered arrangement composes where its arrangement block is absent", "[presets][registry]")
{
    const std::filesystem::path directory = scratch("absent_block");
    const auto registry                   = std::make_shared<scene::preset_registry>();
    presets::register_arrangements(registry, every(directory), {}, {});

    compose_each(*registry);
}

// Every scenario is bound to a document now, so every one of them announces where an edit would
// land -- including the four that read none of the values that document carries.
TEST_CASE("composing an arrangement announces the document it was built from", "[presets][registry]")
{
    const std::filesystem::path directory = scratch("announced");
    const auto registry                   = std::make_shared<scene::preset_registry>();

    std::vector<std::string> announced;
    presets::register_arrangements(registry, every(directory), {},
                                   [&announced](const config::binding &at, const config::document &) { announced.push_back(at.at.resolved.filename().string()); });

    compose_each(*registry);

    REQUIRE(sorted(announced) ==
            std::vector<std::string>{"euler-pose.xml", "euler-relative.xml", "frame-workbench.xml", "rotation-axis.xml", "screw.xml", "twist-axis.xml", "two-pose.xml"});
}

TEST_CASE("with no route a composition loads through the location its preset was registered with", "[presets][registry]")
{
    const std::filesystem::path directory = scratch("registered_location");
    const auto registry                   = std::make_shared<scene::preset_registry>();
    const std::vector<config::location> documents{one(directory, "only.xml")};

    std::vector<std::filesystem::path> built_from;
    presets::register_arrangements(registry, documents, {}, recording(built_from));

    threepp::Scene target;
    REQUIRE(composed(*registry, shipped.front().name, target) != nullptr);
    REQUIRE(built_from == std::vector<std::filesystem::path>{documents.front().resolved});
}

// The second document appears only after the preset is registered, so what this asserts is an order
// of events rather than a path.
TEST_CASE("a composition loads through whatever its route answers at that moment", "[presets][registry]")
{
    const std::filesystem::path first  = scratch("route_first");
    const std::filesystem::path second = scratch("route_second");
    const auto registry                = std::make_shared<scene::preset_registry>();
    const std::vector<config::location> documents{one(first, "moving.xml")};

    std::vector<std::filesystem::path> built_from;
    presets::register_arrangements(registry, documents, preferring(second, first), recording(built_from));

    threepp::Scene target;
    REQUIRE(composed(*registry, shipped.front().name, target) != nullptr);

    const config::location later = one(second, "moving.xml");
    threepp::Scene again;
    REQUIRE(composed(*registry, shipped.front().name, again) != nullptr);

    REQUIRE(built_from == std::vector<std::filesystem::path>{documents.front().resolved, later.resolved});
}

TEST_CASE("a document naming a scenario the set does not carry registers nothing", "[presets][registry]")
{
    const std::filesystem::path directory = scratch("unknown_scenario");

    registers_nothing({authored(directory, "unknown.xml", "Named all the same", "spiral staircase")});
}

TEST_CASE("a document stating no name registers nothing", "[presets][registry]")
{
    const std::filesystem::path directory = scratch("nameless");

    registers_nothing({authored(directory, "empty-name.xml", "", presets::arrangement_scenario_labels().front()), written(directory, "no-preset.xml", "<arrangement/>")});
}

TEST_CASE("two documents stating one name register one preset", "[presets][registry]")
{
    const std::filesystem::path directory = scratch("one_name");
    const auto registry                   = std::make_shared<scene::preset_registry>();
    const std::string taken               = "Screw motion: parameters";
    const std::vector<config::location> documents{authored(directory, "first.xml", taken, presets::arrangement_scenario_labels().front()),
                                                  authored(directory, "second.xml", taken, presets::arrangement_scenario_labels().front())};

    std::vector<std::string> announced;
    const std::vector<std::string> named = presets::register_arrangements(registry, documents, {}, [&announced](const config::binding &at, const config::document &)
                                                                          { announced.push_back(at.at.resolved.filename().string()); });

    REQUIRE(named == std::vector<std::string>{taken});
    REQUIRE(registry->preset_names().size() == 1u);

    threepp::Scene target;
    REQUIRE(composed(*registry, taken, target) != nullptr);
    REQUIRE(announced == std::vector<std::string>{"first.xml"});
}
