#include "demo_machine.h"
#include "scratch_documents.h"

#include "praxis/presets/arm.h"

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
#include <string_view>

using namespace praxis;
using namespace scratch_documents;

namespace {

// A machine carrying three joints and one value under each of the four inverse-kinematics windows,
// under the preview panel and under the curve plot, every one of them different from what its
// settings struct opens at, so a key nothing reads back reads here as the default it was left at.
constexpr std::string_view a_solving_machine = "<machine>\n"
                                               "    <initial>\n"
                                               "        <joint index=\"0\" degrees=\"0\"/>\n"
                                               "        <joint index=\"1\" degrees=\"0\"/>\n"
                                               "        <joint index=\"2\" degrees=\"0\"/>\n"
                                               "    </initial>\n"
                                               "    <ik_seeds>\n"
                                               "        <start index=\"1\" joints=\"0.25 -0.5 0.75\"/>\n"
                                               "    </ik_seeds>\n"
                                               "    <ik_branch mode=\"preview\" figures=\"false\"/>\n"
                                               "    <ik_iterates start=\"3\" mode=\"preview\"/>\n"
                                               "    <ik_convergence angular=\"false\" linear=\"false\"/>\n"
                                               "    <trajectory_preview parameter=\"false\" rate=\"false\" rate_change=\"false\"/>\n"
                                               "    <joint_curves hidden=\"1 3\"/>\n"
                                               "    <path_comparison first=\"0.25 -0.5 0.75\" second=\"-0.25 0.5 -0.75\" joint_space=\"false\" "
                                               "decoupled=\"false\" screw=\"false\" played=\"decoupled\"/>\n"
                                               "</machine>\n";

presets::arm_scenario opened(const scratch &where, std::string_view text)
{
    author(where.seeds() / "machine.xml", text);

    const config::outcome answered = config::load_or_defaults(demo::machine_keyspace(), config::resolve("machine.xml", where.seeds()), config::expectation::partial);
    REQUIRE_FALSE(answered.failure.has_value());

    return demo::read_machine(answered.values, where.seeds());
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
// the machine's own space, so a `declare_windows` line that was never written reads here.
TEST_CASE("every window key path the presets name is declared in the machine keyspace", "[examples][documents]")
{
    const config::declaration shape = demo::machine_keyspace();

    for(const char *path : presets::window_paths::every)
    {
        INFO(path);
        CHECK(declared_under(shape, path));
    }
}

TEST_CASE("a machine's list of starts is read back out from under the path the seed window keeps it at", "[examples][documents]")
{
    const scratch where("a-list-of-starts");
    const presets::arm_scenario read = opened(where, a_solving_machine);

    REQUIRE(read.ik_seeds.seeds.size() == 1u);
    REQUIRE(read.ik_seeds.seeds.front().size() == 3);
    CHECK(read.ik_seeds.seeds.front()[0] == 0.25);
    CHECK(read.ik_seeds.seeds.front()[1] == -0.5);
    CHECK(read.ik_seeds.seeds.front()[2] == 0.75);
}

TEST_CASE("a machine's branch list is read back out from under the path that window keeps it at", "[examples][documents]")
{
    const scratch where("a-branch-list");
    const presets::arm_scenario read = opened(where, a_solving_machine);

    CHECK(read.ik_branch.mode == manipulator::control_mode::preview);
    CHECK_FALSE(read.ik_branch.figures);
}

TEST_CASE("a machine's iterate table is read back out from under the path that window keeps it at", "[examples][documents]")
{
    const scratch where("an-iterate-table");
    const presets::arm_scenario read = opened(where, a_solving_machine);

    // The document counts the starts from one and the window from zero, as the list of starts does.
    CHECK(read.ik_iterates.start == 2u);
    CHECK(read.ik_iterates.mode == manipulator::control_mode::preview);
}

TEST_CASE("a machine's convergence plot is read back out from under the path that window keeps it at", "[examples][documents]")
{
    const scratch where("a-convergence-plot");
    const presets::arm_scenario read = opened(where, a_solving_machine);

    CHECK_FALSE(read.ik_convergence.angular);
    CHECK_FALSE(read.ik_convergence.linear);
}

TEST_CASE("a machine's preview panel is read back out from under the path that window keeps it at", "[examples][documents]")
{
    const scratch where("a-preview-panel");
    const presets::arm_scenario read = opened(where, a_solving_machine);

    CHECK_FALSE(read.trajectory_preview.parameter);
    CHECK_FALSE(read.trajectory_preview.rate);
    CHECK_FALSE(read.trajectory_preview.rate_change);
}

// The document counts the joints from one, as the names the curves are drawn under do, and the
// window from zero.
TEST_CASE("a machine's curve plot is read back out from under the path that window keeps it at", "[examples][documents]")
{
    const scratch where("a-curve-plot");
    const presets::arm_scenario read = opened(where, a_solving_machine);

    CHECK(read.joint_curves.hidden == std::vector<std::size_t>{0u, 2u});
}

// The spellings index the composer table, so a spelling the table does not answer at its own place
// would compose some other scenario than the one the document names.
TEST_CASE("every scenario spelling resolves to its own place in the composer table", "[examples][documents]")
{
    const std::span<const char *const> spellings = demo::arm_scenario_labels();
    REQUIRE(spellings.size() == demo::scenario_count);

    const scratch where("every-spelling");
    for(std::size_t which = 0; which < spellings.size(); ++which)
    {
        const std::string named = "<machine>\n    <preset scenario=\"" + std::string(spellings[which]) + "\"/>\n</machine>\n";
        author(where.seeds() / "machine.xml", named);

        const config::outcome answered = config::load_or_defaults(demo::machine_keyspace(), config::resolve("machine.xml", where.seeds()), config::expectation::partial);
        INFO(spellings[which]);
        REQUIRE_FALSE(answered.failure.has_value());
        CHECK(demo::preset_scenario(answered.values) == which);
    }
}

// The two ends are read back in the radians they are written in, and the three switches and the
// chosen shape beside them, so a machine opening its comparison somewhere other than the pair the
// library answers reads here.
TEST_CASE("a machine's path comparison is read back out from under the path that window keeps it at", "[examples][documents]")
{
    const scratch where("a-path-comparison");
    const presets::arm_scenario read = opened(where, a_solving_machine);

    REQUIRE(read.path_comparison.first.size() == 3);
    CHECK(read.path_comparison.first[0] == 0.25);
    CHECK(read.path_comparison.second[2] == -0.75);
    CHECK_FALSE(read.path_comparison.joint_space);
    CHECK_FALSE(read.path_comparison.decoupled);
    CHECK_FALSE(read.path_comparison.screw);
    CHECK(read.path_comparison.played == manipulator::compared_path::decoupled);
}
