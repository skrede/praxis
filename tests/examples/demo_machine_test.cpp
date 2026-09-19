#include "demo_documents.h"
#include "scratch_documents.h"

#include "praxis/presets/screw_table.h"
#include "praxis/presets/arm_registration.h"

#include "praxis/manipulator/screw_modeling_window.h"

#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <optional>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace scratch_documents;

namespace {

constexpr std::string_view keeping_a_chain = "<arm>\n    <screw_table document=\"screw-table-kr6r.xml\"/>\n</arm>\n";

constexpr std::string_view keeping_nothing = "<arm>\n    <preset name=\"a machine keeping no chain\"/>\n</arm>\n";

// A chain document as one would ship, carrying a comment so a case comparing it against itself also
// asserts that what an author wrote around the values survived.
constexpr std::string_view a_shipped_chain = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                                             "<!-- the line an author wrote -->\n"
                                             "<screw_table>\n"
                                             "    <screws/>\n"
                                             "</screw_table>\n";

// The document a machine is read from, authored where the documents that ship are read from.
config::document machine_at(const scratch &where, std::string_view text)
{
    author(where.seeds() / "machine.xml", text);

    const config::outcome answered = config::load_or_defaults(presets::arm_binding("machine.xml", where.seeds()));
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
}

// The binding a machine document is read through, built the way the registration builds one: the
// published keyspace over whatever the demonstration's own resolver answers for that name.
config::binding read_through(const demo::documents &mine, const std::filesystem::path &named)
{
    const config::location at = mine.composing(named);

    return config::binding{presets::arm_keyspace(), at, config::expectation::partial};
}

// The chain binding the registration builds for one machine: the name that machine's own document
// carried, answered by the same resolver, under the chain's own keyspace. A machine naming nothing
// keeps nothing.
std::optional<config::binding> kept_chain(const config::document &values, const demo::documents &mine)
{
    const expected<std::string, config::error> named = values.text("screw_table/document");
    if(!named || named.value().empty())
        return std::nullopt;

    return config::binding{presets::screw_table_keyspace(), mine.composing(named.value()), config::expectation::partial};
}

// One joint turning about z at the origin, which is enough for a save to carry a row.
manipulator::screw_modeling_window::settings a_chain()
{
    const rigid_motion::capabilities motions = rigid_motion::baseline();

    return manipulator::screw_modeling_window::settings(transform::Identity(),
                                                        std::vector<screw_axis>{motions.screw.screw_axis_from_angular_linear(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero())});
}

}

TEST_CASE("a preset's document is answered from the shipped one while no copy of it stands", "[demo][documents]")
{
    const scratch where("a-preset-document-with-no-copy");
    const demo::documents mine(where.seeds(), where.state());
    author(where.seeds() / "machine.xml", keeping_nothing);

    const config::binding bound = read_through(mine, "machine.xml");

    REQUIRE(bound.at.resolved == std::filesystem::weakly_canonical(where.seeds() / "machine.xml"));
    REQUIRE(bound.shape.space() == presets::arm_keyspace().space());
    REQUIRE(bound.carries == config::expectation::partial);
}

// The same call over the same arguments with a copy made between them, which is the order a run
// that saves for the first time puts them in.
TEST_CASE("a preset's document is answered from the copy once a save has made one", "[demo][documents]")
{
    const scratch where("a-preset-document-answered-twice");
    const demo::documents mine(where.seeds(), where.state());
    author(where.seeds() / "machine.xml", keeping_nothing);

    const config::binding before = read_through(mine, "machine.xml");
    static_cast<void>(mine.writing("machine.xml"));
    const config::binding after = read_through(mine, "machine.xml");

    REQUIRE(before.at.resolved == std::filesystem::weakly_canonical(where.seeds() / "machine.xml"));
    REQUIRE(after.at.resolved == std::filesystem::weakly_canonical(where.state() / "machine.xml"));
}

// No chain document ships beside the machine documents, so the name a machine carries for one stands
// for a place the application writes rather than for a document it reads.
TEST_CASE("the chain a machine keeps is answered under the directory the application writes to", "[demo][documents]")
{
    const scratch where("a-chain-is-kept-in-the-copy");
    const demo::documents mine(where.seeds(), where.state());

    const std::optional<config::binding> kept = kept_chain(machine_at(where, keeping_a_chain), mine);

    REQUIRE(kept.has_value());
    REQUIRE(kept->at.resolved.parent_path() == where.state());
}

TEST_CASE("a machine naming no chain document keeps nothing", "[demo][documents]")
{
    const scratch where("a-machine-keeping-nothing");
    const demo::documents mine(where.seeds(), where.state());

    REQUIRE_FALSE(kept_chain(machine_at(where, keeping_nothing), mine).has_value());
}

// The same machine, read twice with a copy made in between, answers the one place both times: the
// control that keeps a chain has a single destination that does not move under it.
TEST_CASE("the chain is answered in the same place whether or not a copy is already there", "[demo][documents]")
{
    const scratch where("a-chain-answered-the-same-way-twice");
    const demo::documents mine(where.seeds(), where.state());
    const config::document values = machine_at(where, keeping_a_chain);

    const std::optional<config::binding> first = kept_chain(values, mine);

    // The copy a composition's own leaving makes, which is there for every machine read after it.
    author(where.state() / "screw-table-kr6r.xml", a_shipped_chain);
    const std::optional<config::binding> second = kept_chain(values, mine);

    REQUIRE(first.has_value());
    REQUIRE(second.has_value());
    REQUIRE(second->at.resolved == first->at.resolved);
}

// A chain kept through the route a modeling scenario builds lands under the state directory, and the
// directory the documents ship from is left as it was authored.
TEST_CASE("keeping a chain never writes the directory the documents ship from", "[demo][documents]")
{
    const scratch where("keeping-a-chain-never-writes-the-seed");
    const demo::documents mine(where.seeds(), where.state());

    const std::optional<config::binding> kept = kept_chain(machine_at(where, keeping_a_chain), mine);
    REQUIRE(kept.has_value());

    const manipulator::screw_modeling_window::save_route keep = presets::screw_table_route(kept, rigid_motion::baseline().frame);
    REQUIRE(keep);
    keep(presets::screw_table_path, a_chain());

    REQUIRE(bytes_of(where.seeds() / "machine.xml") == std::string(keeping_a_chain));
    REQUIRE(files_in(where.seeds()) == 1u);
    REQUIRE_FALSE(bytes_of(where.state() / "screw-table-kr6r.xml").empty());
}
