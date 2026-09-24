#include "described_arm.h"
#include "scratch_directory.h"

#include "praxis/presets/screw_table.h"
#include "praxis/presets/arm_scenarios.h"
#include "praxis/presets/arm_registration.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/scheduler/strand.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"

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

// Every document here starts an arm of this width, and every described arm is derived at it, so a
// start the document carries is one the description has a joint for.
constexpr std::size_t axes = 6u;

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

// An empty description leaves that leaf out of the document entirely rather than writing an empty
// one, and so does an empty chain: a leaf nobody wrote and a leaf written blank are two of the
// shapes a document arrives in and a case names which one it means.
config::location authored(const std::filesystem::path &directory, const std::string &file, const std::string &name, const std::string &description,
                          const std::string &scenario = spelling(presets::arm_scenario_kind::forward_kinematics), const std::string &chain = "")
{
    const std::string described = description.empty() ? "" : "<description path=\"" + description + "\"/>";
    const std::string keeping   = chain.empty() ? "" : "<screw_table document=\"" + chain + "\"/>";

    return written(directory, file, "<arm><preset name=\"" + name + "\" scenario=\"" + scenario + "\"/>" + described + keeping + "<initial>" + starting_at(axes) + "</initial></arm>");
}

// The same document with both model leaves written out, which is the shape the shipped documents
// arrive in: a model nobody wants is a blank leaf rather than a missing one.
config::location modeled(const std::filesystem::path &directory, const std::string &file, const std::string &tool, const std::string &world)
{
    return written(directory, file,
                   "<arm><preset name=\"Modeled\" scenario=\"" + spelling(presets::arm_scenario_kind::forward_kinematics) +
                           "\"/><description path=\"six.urdf\"/><tool active=\"true\" model=\"" + tool + "\"/><world_object active=\"true\" model=\"" + world + "\"/><initial>" +
                           starting_at(axes) + "</initial></arm>");
}

// Where a document's leaf is read back from, the scenario read out of it rather than the registry.
presets::arm_scenario read_back(const config::location &named, const std::vector<std::filesystem::path> &roots)
{
    const config::outcome read = config::load_or_defaults(config::binding{presets::arm_keyspace(), named, config::expectation::partial});

    return presets::read_arm(read.values, roots);
}

// A document offering no preset leaves the registry as it found it and contributes no name.
void registers_nothing(const std::vector<config::location> &documents)
{
    const auto registry = std::make_shared<scene::preset_registry>();

    REQUIRE(presets::register_arms(registry, documents, {}, {}, {}).empty());
    REQUIRE(registry->preset_names().empty());
}

// The scene is the caller's, because the composition holds it and must not outlive it. A name the
// registry does not carry answers an empty factory, so it is refused here rather than called.
std::shared_ptr<scene::preset> composed(scene::preset_registry &registry, const std::string &name, threepp::Scene &target)
{
    const scene::window_route nowhere             = [](const std::shared_ptr<scene::imgui_window> &) {};
    const scene::preset_registry::factory compose = registry.load_preset(name);
    REQUIRE(compose != nullptr);

    return compose(scene::preset_site{target, scheduler::strand{}, scheduler::strand{}, [](std::string) {}, nowhere, nowhere, {}});
}

// The binding each composition announced it writes back to, in the order the compositions happened.
presets::composed_route recording(std::vector<config::binding> &announced)
{
    return [&announced](const config::binding &at, const config::document &) { announced.push_back(at); };
}

std::vector<std::filesystem::path> written_back(const std::vector<config::binding> &announced)
{
    std::vector<std::filesystem::path> where;
    for(const config::binding &at : announced)
        where.push_back(at.at.resolved);

    return where;
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

// One document registered and composed once, and the binding that composition announced it writes
// back to. The scene is the caller's because the composition holds it and must not outlive it.
config::binding announced_by(const std::vector<config::location> &documents, const std::vector<std::filesystem::path> &roots, threepp::Scene &target)
{
    std::vector<config::binding> announced;
    const auto registry                       = std::make_shared<scene::preset_registry>();
    const std::vector<std::string> registered = presets::register_arms(registry, documents, roots, {}, recording(announced));
    REQUIRE(registered.size() == 1u);
    REQUIRE(composed(*registry, registered.front(), target) != nullptr);
    REQUIRE(announced.size() == 1u);

    return announced.front();
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
    const fixture::described_arm described(axes, "six");
    const std::filesystem::path directory     = scratch("described_arm");
    const std::vector<config::location> named = {authored(directory, "six.xml", "Six axes", "six.urdf")};
    const std::vector<std::filesystem::path> roots{described.directory};

    const auto registry = std::make_shared<scene::preset_registry>();
    REQUIRE(presets::register_arms(registry, named, roots, {}, {}) == std::vector<std::string>{"Six axes"});

    threepp::Scene target;
    REQUIRE(composed(*registry, "Six axes", target) != nullptr);
}

TEST_CASE("the names answered are the ones the span carried, in the order it carried them", "[presets][registry]")
{
    const fixture::described_arm described(axes, "six");
    const std::filesystem::path directory = scratch("arm_ordering");
    const std::vector<config::location> named{authored(directory, "third.xml", "Third", "six.urdf"), authored(directory, "first.xml", "First", "six.urdf"),
                                              authored(directory, "second.xml", "Second", "six.urdf")};
    const std::vector<std::filesystem::path> roots{described.directory};

    const auto registry = std::make_shared<scene::preset_registry>();
    REQUIRE(presets::register_arms(registry, named, roots, {}, {}) == std::vector<std::string>{"Third", "First", "Second"});
    REQUIRE(registry->preset_names() == std::vector<std::string>{"Third", "First", "Second"});
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
    const std::vector<config::location> named{authored(directory, "six.xml", "Six axes", "elsewhere/six.urdf")};

    const auto registry = std::make_shared<scene::preset_registry>();
    REQUIRE(presets::register_arms(registry, named, {}, {}, {}) == std::vector<std::string>{"Six axes"});

    threepp::Scene target;
    REQUIRE(composed(*registry, "Six axes", target) == nullptr);
}

TEST_CASE("a description resolves the same against one root and against two whose first holds nothing", "[presets][registry]")
{
    const fixture::described_arm described(axes, "six");
    const std::filesystem::path directory = scratch("arm_roots");
    const std::filesystem::path barren    = scratch("arm_roots_barren");
    const std::vector<config::location> named{authored(directory, "six.xml", "Six axes", "six.urdf")};

    REQUIRE(opened_through(named, {described.directory}) == opened_through(named, {barren, described.directory}));
}

// The registry assigns into a map, so without the refusal the second document would replace the
// first and leave one entry where two were meant. Which of the two survived is read back out of the
// composition, since both state one name and the name alone cannot tell them apart.
TEST_CASE("two documents stating one name register one preset", "[presets][registry]")
{
    const fixture::described_arm described(axes, "six");
    const std::filesystem::path directory = scratch("arm_one_name");
    const std::string taken               = "Six axes";
    const std::vector<config::location> documents{authored(directory, "first.xml", taken, "six.urdf"),
                                                  authored(directory, "second.xml", taken, "six.urdf", spelling(presets::arm_scenario_kind::tooling))};
    const std::vector<std::filesystem::path> roots{described.directory};

    std::vector<config::binding> announced;
    const auto registry = std::make_shared<scene::preset_registry>();
    REQUIRE(presets::register_arms(registry, documents, roots, {}, recording(announced)) == std::vector<std::string>{taken});
    REQUIRE(registry->preset_names().size() == 1u);

    threepp::Scene target;
    REQUIRE(composed(*registry, taken, target) != nullptr);
    REQUIRE(written_back(announced) == std::vector<std::filesystem::path>{documents.front().resolved});
}

// praxis folds no case, trims no edge and normalizes nothing: a preset name is the bytes the
// document carried, so four documents whose names differ only in those are four presets.
TEST_CASE("names differing only in letter case or in a trailing space are different names", "[presets][registry]")
{
    const fixture::described_arm described(axes, "six");
    const std::filesystem::path directory = scratch("arm_byte_names");
    const std::vector<config::location> documents{authored(directory, "lower.xml", "six axes", "six.urdf"), authored(directory, "upper.xml", "Six Axes", "six.urdf"),
                                                  authored(directory, "bare.xml", "Arm", "six.urdf"), authored(directory, "spaced.xml", "Arm ", "six.urdf")};
    const std::vector<std::filesystem::path> roots{described.directory};

    const auto registry = std::make_shared<scene::preset_registry>();
    REQUIRE(presets::register_arms(registry, documents, roots, {}, {}) == std::vector<std::string>{"six axes", "Six Axes", "Arm", "Arm "});
    REQUIRE(registry->preset_names().size() == 4u);
}

TEST_CASE("a document stating no name registers nothing", "[presets][registry]")
{
    const std::filesystem::path directory = scratch("arm_nameless");

    registers_nothing({authored(directory, "empty-name.xml", "", "six.urdf"), written(directory, "no-preset.xml", "<arm/>")});
}

TEST_CASE("a document naming no description registers nothing", "[presets][registry]")
{
    const std::filesystem::path directory = scratch("arm_undescribed");

    registers_nothing({authored(directory, "undescribed.xml", "Named all the same", "")});
}

// A spelling outside the sealed set is refused by the document's own load, so refusal alone would
// read the same whether or not the registration reads that failure. The two assertions beside it are
// what discriminate: the load names the leaf and the spelling, and the same document under an
// admitted spelling registers -- so the spelling is what the refusal turns on, and the description
// the document carries all the while is not.
TEST_CASE("a document naming a scenario the table does not carry registers nothing", "[presets][registry]")
{
    const fixture::described_arm described(axes, "six");
    const std::filesystem::path directory = scratch("arm_off_the_table");
    const std::vector<std::filesystem::path> roots{described.directory};
    const std::vector<config::location> off{authored(directory, "off.xml", "Six axes", "six.urdf", "spiral staircase")};
    const std::vector<config::location> admitted{authored(directory, "admitted.xml", "Six axes", "six.urdf")};

    const config::outcome read = config::load_or_defaults(config::binding{presets::arm_keyspace(), off.front(), config::expectation::partial});
    REQUIRE(read.failure.has_value());
    INFO(read.failure->message);
    CHECK(read.failure->message.find("scenario") != std::string::npos);
    CHECK(read.failure->message.find("spiral staircase") != std::string::npos);

    const auto refused = std::make_shared<scene::preset_registry>();
    REQUIRE(presets::register_arms(refused, off, roots, {}, {}).empty());

    const auto carried = std::make_shared<scene::preset_registry>();
    REQUIRE(presets::register_arms(carried, admitted, roots, {}, {}) == std::vector<std::string>{"Six axes"});
}

TEST_CASE("with no route a composition loads through the location its preset was registered with", "[presets][registry]")
{
    const fixture::described_arm described(axes, "six");
    const std::filesystem::path directory = scratch("arm_registered_location");
    const std::vector<config::location> documents{authored(directory, "only.xml", "Six axes", "six.urdf")};

    threepp::Scene target;
    REQUIRE(announced_by(documents, {described.directory}, target).at.resolved == documents.front().resolved);
}

// The second document appears only after the preset is registered, so what this asserts is an order
// of events rather than a path.
TEST_CASE("a composition loads through whatever its route answers at that moment", "[presets][registry]")
{
    const fixture::described_arm described(axes, "six");
    const std::filesystem::path first  = scratch("arm_route_first");
    const std::filesystem::path second = scratch("arm_route_second");
    const std::vector<std::filesystem::path> roots{described.directory};
    const std::vector<config::location> documents{authored(first, "moving.xml", "Six axes", "six.urdf")};

    std::vector<config::binding> announced;
    const auto registry = std::make_shared<scene::preset_registry>();
    presets::register_arms(registry, documents, roots, preferring(second, first), recording(announced));

    threepp::Scene target;
    REQUIRE(composed(*registry, "Six axes", target) != nullptr);

    const config::location later = authored(second, "moving.xml", "Six axes", "six.urdf");
    threepp::Scene again;
    REQUIRE(composed(*registry, "Six axes", again) != nullptr);

    REQUIRE(written_back(announced) == std::vector<std::filesystem::path>{documents.front().resolved, later.resolved});
}

// A chain typed into this scenario is what its windows write back, so the document announced is the
// chain's and the space it is announced under is the chain keyspace's rather than the arm's.
TEST_CASE("a supplied chain announces the document the chain is kept in", "[presets][registry]")
{
    const fixture::described_arm described(axes, "six");
    const std::filesystem::path directory = scratch("arm_supplied_chain");
    const config::location chain          = written(directory, "chain.xml", "<screw_table><screws/></screw_table>");
    const std::vector<config::location> documents{authored(directory, "modeling.xml", "Six axes", "six.urdf", spelling(presets::arm_scenario_kind::supplied_chain), "chain.xml")};

    threepp::Scene target;
    const config::binding announced = announced_by(documents, {described.directory}, target);

    REQUIRE(announced.at.resolved == chain.resolved);
    REQUIRE(announced.shape.space() == presets::screw_table_keyspace().space());
}

// The same chain named by a document naming another scenario, which reads no chain at all: the
// document announced is the arm's own. The pair is what says the differing announce is the
// scenario's doing rather than the presence of the leaf naming a chain.
TEST_CASE("a scenario that keeps no chain announces the arm's own document", "[presets][registry]")
{
    const fixture::described_arm described(axes, "six");
    const std::filesystem::path directory = scratch("arm_own_binding");
    written(directory, "chain.xml", "<screw_table><screws/></screw_table>");
    const std::vector<config::location> documents{authored(directory, "forward.xml", "Six axes", "six.urdf", spelling(presets::arm_scenario_kind::forward_kinematics), "chain.xml")};

    threepp::Scene target;
    const config::binding announced = announced_by(documents, {described.directory}, target);

    REQUIRE(announced.at.resolved == documents.front().resolved);
    REQUIRE(announced.shape.space() == presets::arm_keyspace().space());
}

// An arm naming the scenario and no chain document keeps nothing: the composition is answered all
// the same, what it announces is its own document, and the chain its windows open at is one nobody
// supplied.
TEST_CASE("a supplied chain naming no document keeps nothing and announces its own", "[presets][registry]")
{
    const fixture::described_arm described(axes, "six");
    const std::filesystem::path directory = scratch("arm_chainless");
    const std::vector<config::location> documents{authored(directory, "modeling.xml", "Six axes", "six.urdf", spelling(presets::arm_scenario_kind::supplied_chain))};

    threepp::Scene target;
    const config::binding announced = announced_by(documents, {described.directory}, target);

    REQUIRE(announced.at.resolved == documents.front().resolved);
    REQUIRE(announced.shape.space() == presets::arm_keyspace().space());
}

// Nothing deployed and nothing vendored: the description is an arm this case synthesized into a
// directory of its own, and the roots span is the only place its path is looked for. The second half
// is the same two documents with that span emptied -- the names still come back, each composition
// answers nothing rather than a half-built scenario, and the path the failure would name is the one
// the document wrote rather than a root that was never chosen.
TEST_CASE("two arms register and compose against a description no root but the case's own holds", "[presets][registry]")
{
    const fixture::described_arm described(axes, "six");
    const std::filesystem::path directory = scratch("arm_headless");
    const std::vector<config::location> documents{authored(directory, "first.xml", "First", "six.urdf"), authored(directory, "second.xml", "Second", "six.urdf")};
    const std::vector<std::string> both{"First", "Second"};

    const auto held = std::make_shared<scene::preset_registry>();
    REQUIRE(presets::register_arms(held, documents, std::vector<std::filesystem::path>{described.directory}, {}, {}) == both);

    const auto unheld = std::make_shared<scene::preset_registry>();
    REQUIRE(presets::register_arms(unheld, documents, {}, {}, {}) == both);

    for(const std::string &name : both)
    {
        INFO(name);
        threepp::Scene target;
        REQUIRE(composed(*held, name, target) != nullptr);

        threepp::Scene without;
        REQUIRE(composed(*unheld, name, without) == nullptr);
    }

    const config::outcome read = config::load_or_defaults(config::binding{presets::arm_keyspace(), documents.front(), config::expectation::partial});
    REQUIRE(presets::read_arm(read.values, {}).description == std::filesystem::path("six.urdf"));
}

// The two models a document names are looked for where its description is looked for, so a scenario
// read from somewhere other than the directory the binary was started in names files that are there.
// A leaf written blank stays blank, because a root joined onto nothing is the root directory itself
// and a directory is not a model.
TEST_CASE("the model paths a document names resolve against the roots its description does", "[presets][registry]")
{
    const std::filesystem::path directory = scratch("arm_model_paths");
    const std::filesystem::path root      = scratch("arm_model_root");
    std::filesystem::create_directories(root / "meshes");
    written(root / "meshes", "gripper.stl", "solid gripper endsolid gripper");
    written(root / "meshes", "table.stl", "solid table endsolid table");

    const config::location named = modeled(directory, "tooled.xml", "meshes/gripper.stl", "meshes/table.stl");
    const std::vector<std::filesystem::path> one{root};

    const presets::arm_scenario held = read_back(named, one);
    REQUIRE(held.tool.model_path == (root / "meshes" / "gripper.stl").string());
    REQUIRE(held.world_object.model_path == (root / "meshes" / "table.stl").string());

    const presets::arm_scenario unheld = read_back(named, {});
    REQUIRE(unheld.tool.model_path == "meshes/gripper.stl");
    REQUIRE(unheld.world_object.model_path == "meshes/table.stl");

    const presets::arm_scenario blank = read_back(modeled(directory, "untooled.xml", "", ""), one);
    REQUIRE(blank.tool.model_path.empty());
    REQUIRE(blank.world_object.model_path.empty());
}
