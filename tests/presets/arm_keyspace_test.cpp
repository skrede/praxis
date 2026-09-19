#include "scratch_directory.h"

#include "praxis/presets/arm.h"
#include "praxis/presets/arm_scenarios.h"
#include "praxis/presets/arm_registration.h"

#include "praxis/manipulator/control_mode.h"
#include "praxis/manipulator/path_comparison_window.h"

#include "praxis/config/store.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <optional>
#include <string_view>
#include <filesystem>

using namespace praxis;

namespace {

// Three joints and one value under each of the windows read back below, every one of them different
// from what its settings struct opens at, so a key nothing reads back reads here as the default.
constexpr std::string_view a_solving_arm = "<arm>\n"
                                           "    <initial><joint index=\"0\" degrees=\"0\"/><joint index=\"1\" degrees=\"0\"/><joint index=\"2\" degrees=\"0\"/></initial>\n"
                                           "    <ik_seeds><start index=\"1\" joints=\"0.25 -0.5 0.75\"/></ik_seeds>\n"
                                           "    <ik_branch mode=\"preview\" figures=\"false\"/><ik_iterates start=\"3\" mode=\"preview\"/>\n"
                                           "    <ik_convergence angular=\"false\" linear=\"false\"/><joint_curves hidden=\"1 3\"/>\n"
                                           "    <trajectory_preview parameter=\"false\" rate=\"false\" rate_change=\"false\"/>\n"
                                           "    <path_comparison first=\"0.25 -0.5 0.75\" second=\"-0.25 0.5 -0.75\" joint_space=\"false\" "
                                           "decoupled=\"false\" screw=\"false\" played=\"decoupled\"/>\n"
                                           "</arm>\n";

std::filesystem::path scratch(const char *named)
{
    const std::filesystem::path directory = fixture::shared_scratch_directory() / named;
    std::filesystem::create_directories(directory);

    return directory;
}

void author(const std::filesystem::path &file, std::string_view text)
{
    std::ofstream out(file, std::ios::binary | std::ios::trunc);
    out << text;
}

config::outcome loaded(const std::filesystem::path &where, std::string_view text)
{
    author(where / "arm.xml", text);

    return config::load_or_defaults(presets::arm_keyspace(), config::resolve("arm.xml", where), config::expectation::partial);
}

presets::arm_scenario opened(const char *named, std::string_view text)
{
    const config::outcome answered = loaded(scratch(named), text);
    REQUIRE_FALSE(answered.failure.has_value());

    return presets::read_arm(answered.values, {});
}

// A path is declared where the keyspace carries a node at it or anywhere beneath it, which is what a
// group with only leaves under it looks like.
bool declared_under(const config::declaration &shape, std::string_view path)
{
    for(const config::node &held : shape.nodes())
        if(held.path == path || (held.path.starts_with(path) && held.path.size() > path.size() && held.path[path.size()] == '/'))
            return true;

    return false;
}

}

// The table names every path a window keeps settings under; a path nothing declared has no key in
// the arm's own space, so a declaration line that was never written reads here.
TEST_CASE("every window key path the presets name is declared in the arm keyspace", "[presets][documents]")
{
    const config::declaration shape = presets::arm_keyspace();

    for(const char *path : presets::window_paths::every)
    {
        INFO(path);
        CHECK(declared_under(shape, path));
    }
}

TEST_CASE("an arm's list of starts is read back out from under the path the seed window keeps it at", "[presets][documents]")
{
    const presets::arm_scenario read = opened("a-list-of-starts", a_solving_arm);

    REQUIRE(read.ik_seeds.seeds.size() == 1u);
    REQUIRE(read.ik_seeds.seeds.front().size() == 3);
    CHECK(read.ik_seeds.seeds.front()[0] == 0.25);
    CHECK(read.ik_seeds.seeds.front()[1] == -0.5);
    CHECK(read.ik_seeds.seeds.front()[2] == 0.75);
}

TEST_CASE("an arm's branch list is read back out from under the path that window keeps it at", "[presets][documents]")
{
    const presets::arm_scenario read = opened("a-branch-list", a_solving_arm);

    CHECK(read.ik_branch.mode == manipulator::control_mode::preview);
    CHECK_FALSE(read.ik_branch.figures);
}

TEST_CASE("an arm's iterate table is read back out from under the path that window keeps it at", "[presets][documents]")
{
    const presets::arm_scenario read = opened("an-iterate-table", a_solving_arm);

    // The document counts the starts from one and the window from zero, as the list of starts does.
    CHECK(read.ik_iterates.start == 2u);
    CHECK(read.ik_iterates.mode == manipulator::control_mode::preview);
}

TEST_CASE("an arm's convergence plot is read back out from under the path that window keeps it at", "[presets][documents]")
{
    const presets::arm_scenario read = opened("a-convergence-plot", a_solving_arm);

    CHECK_FALSE(read.ik_convergence.angular);
    CHECK_FALSE(read.ik_convergence.linear);
}

TEST_CASE("an arm's preview panel is read back out from under the path that window keeps it at", "[presets][documents]")
{
    const presets::arm_scenario read = opened("a-preview-panel", a_solving_arm);

    CHECK_FALSE(read.trajectory_preview.parameter);
    CHECK_FALSE(read.trajectory_preview.rate);
    CHECK_FALSE(read.trajectory_preview.rate_change);
}

// The document counts the joints from one, as the curves' own names do, and the window from zero.
TEST_CASE("an arm's curve plot is read back out from under the path that window keeps it at", "[presets][documents]")
{
    const presets::arm_scenario read = opened("a-curve-plot", a_solving_arm);

    CHECK(read.joint_curves.hidden == std::vector<std::size_t>{0u, 2u});
}

// The two ends are read back in the radians they are written in, and the three switches and the
// chosen shape beside them, so an arm opening its comparison at some other pair reads here.
TEST_CASE("an arm's path comparison is read back out from under the path that window keeps it at", "[presets][documents]")
{
    const presets::arm_scenario read = opened("a-path-comparison", a_solving_arm);

    REQUIRE(read.path_comparison.first.size() == 3);
    CHECK(read.path_comparison.first[0] == 0.25);
    CHECK(read.path_comparison.second[2] == -0.75);
    CHECK_FALSE(read.path_comparison.joint_space);
    CHECK_FALSE(read.path_comparison.decoupled);
    CHECK_FALSE(read.path_comparison.screw);
    CHECK(read.path_comparison.played == manipulator::compared_path::decoupled);
}

// The spellings stand in the enumeration's own order, so one answered at its neighbour's place would
// compose some other scenario than the one the document names.
TEST_CASE("every scenario spelling resolves to its own enumerator", "[presets][documents]")
{
    const std::span<const char *const> spellings = presets::arm_scenario_labels();
    const std::filesystem::path where            = scratch("every-spelling");

    for(std::size_t which = 0u; which < spellings.size(); ++which)
    {
        const config::outcome answered = loaded(where, "<arm>\n    <preset scenario=\"" + std::string(spellings[which]) + "\"/>\n</arm>\n");
        INFO(spellings[which]);
        REQUIRE_FALSE(answered.failure.has_value());
        CHECK(presets::arm_scenario_named(answered.values) == static_cast<presets::arm_scenario_kind>(which));
    }
}

TEST_CASE("a spelling the table does not carry is declined by the load", "[presets][documents]")
{
    const config::outcome answered = loaded(scratch("off-the-table"), "<arm>\n    <preset scenario=\"spiral staircase\"/>\n</arm>\n");

    CHECK(answered.failure.has_value());
}

// Refusing an off-table spelling is only half of it: a document naming no scenario at all answers
// nothing rather than the first enumerator, which a read against another declaration stands for.
TEST_CASE("a document read against another declaration names no scenario", "[presets][documents]")
{
    const std::filesystem::path where = scratch("another-declaration");
    author(where / "elsewhere.xml", "<elsewhere>\n    <preset label=\"named\"/>\n</elsewhere>\n");

    config::declaration other("elsewhere");
    other.group("preset");
    other.field("preset/label", config::field_kind::text, "");
    const config::outcome answered = config::load_or_defaults(other, config::resolve("elsewhere.xml", where), config::expectation::partial);
    REQUIRE_FALSE(answered.failure.has_value());
    CHECK(presets::arm_scenario_named(answered.values) == std::nullopt);
}
