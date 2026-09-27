#include "scratch_directory.h"

#include "praxis/presets/screw_table.h"

#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/screw_modeling_window.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <utility>
#include <iterator>
#include <optional>
#include <algorithm>
#include <filesystem>
#include <string_view>

using namespace praxis;

namespace {

using supplied = manipulator::screw_modeling_window::settings;

constexpr std::string_view at = presets::screw_table_path;

const rigid_motion::capabilities &motions()
{
    static const rigid_motion::capabilities bound = rigid_motion::baseline();

    return bound;
}

const std::filesystem::path &scratch()
{
    return fixture::shared_scratch_directory();
}

screw_axis six_vector(double from)
{
    screw_axis named;
    for(Eigen::Index component = 0; component < 6; ++component)
        named[component] = from + 0.125 * static_cast<double>(component);

    return named;
}

// Five joints that turn and one that only translates, so the opening state is not one value
// repeated and a row filled with the wrong one is visible.
manipulator::screw_chain derived_six()
{
    manipulator::screw_chain chain;
    for(std::size_t joint = 0u; joint < 5u; ++joint)
        chain.space_screws.push_back(motions().screw.screw_axis_from_angular_linear(Eigen::Vector3d::UnitY(), Eigen::Vector3d(0.0, 0.0, 0.1 * static_cast<double>(joint + 1u))));
    chain.space_screws.push_back(motions().screw.screw_axis_from_angular_linear(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX()));

    return chain;
}

// Three joints that all turn, for a document written by hand where what is under test is the order
// its rows stand in rather than which construction each of them opens in.
manipulator::screw_chain derived_three()
{
    manipulator::screw_chain chain;
    for(std::size_t joint = 0u; joint < 3u; ++joint)
        chain.space_screws.push_back(motions().screw.screw_axis_from_angular_linear(Eigen::Vector3d::UnitY(), Eigen::Vector3d(0.0, 0.0, 0.1 * static_cast<double>(joint + 1u))));

    return chain;
}

std::string keys_of_rows()
{
    return std::string(at) + "/joint";
}

// A distinguishable value in every leaf the table declares, so a leaf read back as what was written
// there cannot be a fallback that happens to agree with it.
supplied a_chain(std::size_t joints)
{
    supplied chosen;
    chosen.home = motions().frame.transformation_matrix_from_rotation_position(
            motions().frame.rotation_matrix_from_euler(Eigen::Vector3d(0.2, -0.3, 0.4), manipulator::screw_modeling_window::home_axis_order), Eigen::Vector3d(0.125, -0.25, 0.375));
    for(std::size_t joint = 0u; joint < joints; ++joint)
        chosen.screws.push_back(six_vector(1.0 + static_cast<double>(joint)));

    return chosen;
}

config::binding binding_at(const char *name)
{
    std::error_code ignored;
    std::filesystem::remove(scratch() / name, ignored);

    return presets::screw_table_binding(name, scratch());
}

config::document carried(const config::binding &bound)
{
    return config::load_or_defaults(bound).values;
}

// A document written out by hand and then bound, so the binding opens over rows the suite chose the
// order of rather than over a file a writer just laid down.
config::binding binding_over(const std::string &body, const char *name)
{
    const std::filesystem::path where = scratch() / name;
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<screw_table><screws>" << body << "</screws></screw_table>\n";
    out.close();

    return presets::screw_table_binding(name, scratch());
}

config::document authored(const std::string &body, const char *name)
{
    const std::filesystem::path where = scratch() / name;
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<screw_table><screws>" << body << "</screws></screw_table>\n";
    out.close();

    const expected<config::document, config::error> read = config::load(presets::screw_table_keyspace(), config::resolve(where, scratch()));
    INFO((read ? std::string() : read.error().message));
    REQUIRE(read.has_value());

    return read.value();
}

std::string triple(std::string_view named, const Eigen::Vector3d &value)
{
    return "<" + std::string(named) + " x=\"" + std::to_string(value.x()) + "\" y=\"" + std::to_string(value.y()) + "\" z=\"" + std::to_string(value.z()) + "\"/>";
}

std::string row(std::string_view index, const std::optional<screw_axis> &screw)
{
    std::string element = "<joint index=\"" + std::string(index) + "\">";
    if(screw)
        element += triple("angular", screw->head<3>()) + triple("linear", screw->tail<3>());

    return element + "</joint>";
}

std::string row(std::size_t index, const std::optional<screw_axis> &screw)
{
    return row(std::to_string(index), screw);
}

std::string rows_through(std::size_t joints)
{
    std::string body;
    for(std::size_t joint = 0u; joint < joints; ++joint)
        body += row(joint + 1u, six_vector(1.0 + static_cast<double>(joint)));

    return body;
}

supplied opened(const config::document &values, const manipulator::screw_chain &derived)
{
    const expected<supplied, config::error> read = presets::read_screw_table(values, at, derived, motions().screw, motions().frame);
    INFO((read ? std::string() : read.error().message));
    REQUIRE(read.has_value());

    return read.value();
}

// What the writer answers for a state it has to accept.
std::vector<config::edit> written(const config::document &values, const manipulator::screw_chain &derived, const supplied &state)
{
    const expected<std::vector<config::edit>, config::error> changes = presets::write_screw_table(values, at, derived, state, motions().frame);
    INFO((changes ? std::string() : changes.error().message));
    REQUIRE(changes.has_value());

    return changes.value();
}

std::string bytes_at(const char *name)
{
    std::ifstream in(scratch() / name, std::ios::binary);

    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

// The document at `name` rewritten with every line ending in a carriage return and a line feed.
void ended_with_carriage_returns(const char *name)
{
    std::string ended;
    for(const char letter : bytes_at(name))
    {
        if(letter == '\n')
            ended.push_back('\r');
        ended.push_back(letter);
    }

    std::ofstream out(scratch() / name, std::ios::binary | std::ios::trunc);
    out << ended;
}

std::size_t occurrences(const std::string &text, std::string_view needle)
{
    std::size_t counted = 0u;
    for(std::size_t found = text.find(needle); found != std::string::npos; found = text.find(needle, found + needle.size()))
        ++counted;

    return counted;
}

std::string with_line_feeds_only(const std::string &text)
{
    std::string fed;
    for(std::size_t letter = 0u; letter < text.size(); ++letter)
        if(text[letter] != '\r' || letter + 1u >= text.size() || text[letter + 1u] != '\n')
            fed.push_back(text[letter]);

    return fed;
}

void require_same_screws(const supplied &read, const supplied &chosen)
{
    REQUIRE(read.screws.size() == chosen.screws.size());
    for(std::size_t joint = 0u; joint < chosen.screws.size(); ++joint)
    {
        INFO("joint " << joint);
        REQUIRE(read.screws[joint].has_value());
        REQUIRE(chosen.screws[joint].has_value());
        REQUIRE((*read.screws[joint] - *chosen.screws[joint]).norm() == 0.0);
    }
}

// Whether a chain holds a screw for one joint, which one it does not stretch to never does.
bool holds(const supplied &chain, std::size_t joint)
{
    return joint < chain.screws.size() && chain.screws[joint].has_value();
}

struct shaped
{
    const char *named;
    std::string body;
    bool carried;
};

// Every shape a document hands the writer: none at all, in the chain's order, out of it, with holes,
// naming a joint past the chain's end, and carrying instances that hold no leaf.
std::vector<shaped> document_shapes()
{
    std::string leafless;
    for(std::size_t joint = 1u; joint <= 6u; ++joint)
        leafless += row(joint, std::nullopt);

    return {shaped{"absent", "", false},
            shaped{"in-order", rows_through(6u), true},
            shaped{"out-of-order", row(3u, six_vector(3.0)) + row(1u, six_vector(1.0)) + row(2u, six_vector(2.0)), true},
            shaped{"sparse", row(2u, six_vector(2.0)) + row(5u, six_vector(5.0)), true},
            shaped{"surplus", rows_through(7u), true},
            shaped{"leafless", leafless, true}};
}

// Every shape of state a caller hands the writer, against a chain of six joints.
std::vector<std::pair<const char *, supplied>> state_shapes()
{
    supplied empty;
    empty.screws.resize(6u);

    supplied alternating = a_chain(6u);
    for(std::size_t joint = 1u; joint < alternating.screws.size(); joint += 2u)
        alternating.screws[joint].reset();

    supplied zeros;
    zeros.screws.assign(6u, screw_axis::Zero());

    return {{"held", a_chain(6u)}, {"empty", empty}, {"alternating", alternating}, {"zeros", zeros}, {"past-the-end", a_chain(7u)}, {"shorter", a_chain(3u)}};
}
}

TEST_CASE("a chain written into a document reads back as the chain it was", "[presets][configuration]")
{
    const config::binding bound = binding_at("round-trip.xml");
    const supplied chosen       = a_chain(6);

    const std::vector<config::edit> changes = written(carried(bound), derived_six(), chosen);
    REQUIRE(changes.size() == 6u + 6u * 7u);
    REQUIRE(config::save(bound, changes).has_value());

    const supplied read = opened(carried(bound), derived_six());
    REQUIRE((read.home - chosen.home).norm() < 1.0e-5);
    require_same_screws(read, chosen);
}

TEST_CASE("a chain no document names leaves every joint unsupplied and its home at the identity", "[presets][configuration]")
{
    const config::outcome absent = config::load_or_defaults(binding_at("no-chain-was-kept.xml"));
    REQUIRE(absent.failure.has_value());

    const supplied read = opened(absent.values, derived_six());
    REQUIRE((read.home - transform::Identity()).norm() == 0.0);
    REQUIRE(read.screws.size() == 6u);
    for(std::size_t joint = 0u; joint < read.screws.size(); ++joint)
    {
        INFO("joint " << joint);
        REQUIRE_FALSE(read.screws[joint].has_value());
    }
}

TEST_CASE("a document carrying only some of the chain's rows leaves the rest unsupplied", "[presets][configuration]")
{
    const supplied read = opened(authored(rows_through(3u), "some-rows.xml"), derived_six());
    REQUIRE(read.screws.size() == 6u);
    for(std::size_t joint = 0u; joint < 3u; ++joint)
    {
        INFO("joint " << joint);
        REQUIRE(read.screws[joint].has_value());
        REQUIRE((*read.screws[joint] - six_vector(1.0 + static_cast<double>(joint))).norm() < 1.0e-5);
    }
    for(std::size_t joint = 3u; joint < read.screws.size(); ++joint)
    {
        INFO("joint " << joint);
        REQUIRE_FALSE(read.screws[joint].has_value());
    }
}

// The screw a row nobody supplied opens at is one somebody could plausibly have written down, so
// the two entries carry equal numbers and only where each came from tells them apart.
TEST_CASE("a row naming exactly the numbers a row nobody supplied opens at is supplied, and its neighbour is not", "[presets][configuration]")
{
    const manipulator::screw_chain derived = derived_six();
    const screw_axis padding               = manipulator::screw_modeling_window::opening_screw(motions().screw, derived.space_screws.front());

    const supplied read = opened(authored(row(1u, padding), "padding-equal.xml"), derived);

    REQUIRE(read.screws.size() == 6u);
    REQUIRE(read.screws.front().has_value());
    CHECK((*read.screws.front() - padding).norm() < 1.0e-5);
    CHECK_FALSE(read.screws[1].has_value());
}

// A row is resolved by the identity it carries rather than by where it stands, so the order the
// document lists its joints in is not the order they are read into.
TEST_CASE("a table naming its joints out of the chain's order lands each screw at its own joint", "[presets][configuration]")
{
    const std::string body = row(3u, six_vector(3.0)) + row(1u, six_vector(1.0)) + row(2u, six_vector(2.0));

    const supplied read = opened(authored(body, "named-out-of-order.xml"), derived_six());

    REQUIRE(read.screws.size() == 6u);
    for(std::size_t joint = 0u; joint < 3u; ++joint)
    {
        INFO("joint " << joint);
        REQUIRE(read.screws[joint].has_value());
        CHECK((*read.screws[joint] - six_vector(1.0 + static_cast<double>(joint))).norm() < 1.0e-5);
    }
    for(std::size_t joint = 3u; joint < read.screws.size(); ++joint)
    {
        INFO("joint " << joint);
        CHECK_FALSE(read.screws[joint].has_value());
    }
}

// An instance present is a row somebody wrote, whatever it leaves out, and a leaf left out is the
// zero the declaration falls back to -- which is exactly what a writer emitting only what moved
// leaves behind.
TEST_CASE("a row the document carries an instance of with every leaf left out is supplied at zero", "[presets][configuration]")
{
    const supplied read = opened(authored(row(2u, std::nullopt), "a-bare-instance.xml"), derived_six());

    REQUIRE(read.screws.size() == 6u);
    REQUIRE(read.screws[1].has_value());
    CHECK(read.screws[1]->norm() == 0.0);
    CHECK_FALSE(read.screws.front().has_value());
}

TEST_CASE("a document naming no joint at all leaves every joint unsupplied", "[presets][configuration]")
{
    const supplied read = opened(authored(std::string(), "names-no-joint.xml"), derived_six());

    REQUIRE(read.screws.size() == 6u);
    for(std::size_t joint = 0u; joint < read.screws.size(); ++joint)
    {
        INFO("joint " << joint);
        CHECK_FALSE(read.screws[joint].has_value());
    }
}

// A chain kept for a longer machine is read against this one row by row, and every row lands at the
// joint its own identity names: a surplus is carried at the far end rather than shifted into the
// joints this chain does have.
TEST_CASE("a table naming two joints more than the chain has lands every row at the joint it names", "[presets][configuration]")
{
    const supplied read = opened(authored(rows_through(8u), "wrong-length.xml"), derived_six());

    REQUIRE(read.screws.size() == 8u);
    for(std::size_t joint = 0u; joint < 8u; ++joint)
    {
        INFO("joint " << joint);
        REQUIRE(read.screws[joint].has_value());
        CHECK((*read.screws[joint] - six_vector(1.0 + static_cast<double>(joint))).norm() < 1.0e-5);
    }
}

// The length a surplus stretches the reading to is the highest joint any row names, and the joints
// between the chain's end and that row are joints nobody supplied rather than rows moved up.
TEST_CASE("a table naming a joint past the chain's end and nothing between leaves the joints between unsupplied", "[presets][configuration]")
{
    const supplied read = opened(authored(row(1u, six_vector(1.0)) + row(7u, six_vector(2.0)), "out-of-order.xml"), derived_six());

    REQUIRE(read.screws.size() == 7u);
    REQUIRE(read.screws.front().has_value());
    CHECK((*read.screws.front() - six_vector(1.0)).norm() < 1.0e-5);
    REQUIRE(read.screws.back().has_value());
    CHECK((*read.screws.back() - six_vector(2.0)).norm() < 1.0e-5);
    for(std::size_t joint = 1u; joint < 6u; ++joint)
    {
        INFO("joint " << joint);
        CHECK_FALSE(read.screws[joint].has_value());
    }
}

// Somebody who derives one screw more than the arm has joints has written down a chain this machine
// cannot be, and the row past the last joint is the whole of what says so. It is read like any other
// row and carried, so the count the reading states is the count the document named.
TEST_CASE("a table naming one joint more than the chain has is read, and the surplus row is carried", "[presets][configuration]")
{
    const supplied read = opened(authored(rows_through(7u), "one-row-too-many.xml"), derived_six());

    REQUIRE(read.screws.size() == 7u);
    REQUIRE(read.screws.back().has_value());
    CHECK((*read.screws.back() - six_vector(7.0)).norm() < 1.0e-5);
}

// The boundary the surplus is measured against: a document naming every joint and no more is a
// complete chain, and one entry past it would make the reading say a surplus nobody wrote.
TEST_CASE("a table naming exactly as many joints as the chain has reads that many entries and no more", "[presets][configuration]")
{
    const supplied read = opened(authored(rows_through(6u), "exactly-the-chain.xml"), derived_six());

    REQUIRE(read.screws.size() == 6u);
    REQUIRE(read.screws.back().has_value());
    CHECK((*read.screws.back() - six_vector(6.0)).norm() < 1.0e-5);
}

// An identity no reading makes a joint of is not one to guess at: dropping the row would put
// something somebody wrote out of reach with nothing said anywhere, so the document is turned away
// by the identity itself.
TEST_CASE("a table whose row identity is not an ordinal is refused, naming that identity", "[presets][configuration]")
{
    const expected<supplied, config::error> read =
            presets::read_screw_table(authored(row("wrist", six_vector(1.0)), "not-an-ordinal.xml"), at, derived_six(), motions().screw, motions().frame);
    REQUIRE_FALSE(read.has_value());

    INFO(read.error().message);
    CHECK(read.error().message.find("wrist") != std::string::npos);
}

// A row is looked up by the text it carries, so an identity that reads as a joint's place without
// being spelled the way that place reads back is an identity no lookup can answer. It is turned
// away by the identity itself rather than admitted and then found by nothing.
TEST_CASE("a table whose row identity carries a leading zero is refused, naming that identity", "[presets][configuration]")
{
    const expected<supplied, config::error> read =
            presets::read_screw_table(authored(row("07", six_vector(1.0)), "a-padded-ordinal.xml"), at, derived_six(), motions().screw, motions().frame);
    REQUIRE_FALSE(read.has_value());

    INFO(read.error().message);
    CHECK(read.error().message.find("07") != std::string::npos);
}

// A document may name joints past the end of the chain it is read against -- that surplus is how a
// chain kept for a longer machine says so -- but only so far, and the far side of that is a row the
// reading refuses by name rather than one it stretches to hold.
TEST_CASE("a table naming the furthest joint a surplus reaches is read to it and one past it is refused", "[presets][configuration]")
{
    const std::size_t furthest = 6u + presets::screw_table_greatest_surplus;
    const supplied read        = opened(authored(row(furthest, six_vector(2.0)), "the-furthest-surplus.xml"), derived_six());
    REQUIRE(read.screws.size() == furthest);
    REQUIRE(read.screws.back().has_value());
    CHECK((*read.screws.back() - six_vector(2.0)).norm() < 1.0e-5);

    const std::string beyond = std::to_string(furthest + 1u);
    const expected<supplied, config::error> past =
            presets::read_screw_table(authored(row(beyond, six_vector(2.0)), "past-the-surplus.xml"), at, derived_six(), motions().screw, motions().frame);
    REQUIRE_FALSE(past.has_value());

    INFO(past.error().message);
    CHECK(past.error().message.find(beyond) != std::string::npos);
}

// An ordinal no chain reaches is a typo rather than a longer machine, and the reading it asks for is
// one no process holds. It is refused before anything is allocated for it, so the document meets an
// answer rather than the reader running out of memory inside a call that promises one.
TEST_CASE("a table naming an ordinal no chain could reach is refused rather than allocated for", "[presets][configuration]")
{
    const expected<supplied, config::error> read =
            presets::read_screw_table(authored(row("18446744073709551615", six_vector(1.0)), "an-unreachable-ordinal.xml"), at, derived_six(), motions().screw, motions().frame);
    REQUIRE_FALSE(read.has_value());

    INFO(read.error().message);
    CHECK(read.error().message.find("18446744073709551615") != std::string::npos);
}

TEST_CASE("a chain written twice with nothing moved between offers no second edit", "[presets][configuration]")
{
    const config::binding bound = binding_at("nothing-moved.xml");
    const supplied chosen       = a_chain(6);

    REQUIRE(config::save(bound, written(carried(bound), derived_six(), chosen)).has_value());
    REQUIRE(written(carried(bound), derived_six(), chosen).empty());
}

TEST_CASE("a row the document carries no instance of is appended at the collection's current size, named before it is filled", "[presets][configuration]")
{
    const config::document values = authored(row(2u, six_vector(2.5)) + row(3u, six_vector(3.5)), "appended.xml");
    const std::string rows        = std::string(at) + "/joint";

    const std::vector<config::edit> changes = written(values, derived_three(), a_chain(3));
    REQUIRE(changes.size() == 6u + 7u + 6u + 6u);

    REQUIRE(changes[6].key == rows + "[2]/index");
    REQUIRE(changes[6].value == "1");
    REQUIRE(changes[7].key == rows + "[2]/angular/x");
    REQUIRE(changes[13].key == rows + "[0]/angular/x");
    REQUIRE(changes[19].key == rows + "[1]/angular/x");
}

TEST_CASE("a route writes the chain it is handed into the binding's document, and a binding naming no document answers none", "[presets][configuration]")
{
    REQUIRE_FALSE(presets::screw_table_route(std::nullopt, derived_six(), motions().frame));
    REQUIRE_FALSE(presets::screw_table_route(config::binding{presets::screw_table_keyspace(), config::location{}, config::expectation::partial}, derived_six(), motions().frame));

    const config::binding bound                               = binding_at("routed.xml");
    const manipulator::screw_modeling_window::save_route keep = presets::screw_table_route(bound, derived_six(), motions().frame);
    REQUIRE(keep);

    const supplied chosen = a_chain(6);
    keep(at, chosen);

    require_same_screws(opened(carried(bound), derived_six()), chosen);
}

// The route reads the document again where the chain arrives rather than keeping the one it was
// composed over, so a second save writes into the rows the first appended instead of beside them.
TEST_CASE("a route saved twice leaves the document holding one row per joint", "[presets][configuration]")
{
    const config::binding bound                               = binding_at("routed-twice.xml");
    const manipulator::screw_modeling_window::save_route keep = presets::screw_table_route(bound, derived_six(), motions().frame);
    REQUIRE(keep);

    const supplied chosen = a_chain(6);
    keep(at, chosen);
    keep(at, chosen);

    REQUIRE(carried(bound).identities(keys_of_rows()).size() == chosen.screws.size());
    require_same_screws(opened(carried(bound), derived_six()), chosen);
}

// The window's own offer on leaving is this writer, so a document just written from a chain reports
// nothing outstanding against it -- including the identity leaf, which the keyspace declares as a
// collection's key rather than as a leaf of its own and which no comparison can therefore read.
TEST_CASE("a chain written into a document leaves that document reporting nothing unsaved", "[presets][configuration]")
{
    const config::binding bound = binding_at("converged.xml");
    const supplied chosen       = a_chain(6);

    REQUIRE(config::save(bound, written(carried(bound), derived_six(), chosen)).has_value());

    const manipulator::screw_modeling_window::edit_route spelling = presets::screw_table_edits(derived_six(), motions().frame);
    REQUIRE(spelling);
    CHECK(spelling(carried(bound), at, chosen).empty());
}

// A document somebody wrote by hand names its joints in whatever order it likes, and the reader
// resolves a row by the identity it carries; saving over it must not renumber those rows.
TEST_CASE("a table whose rows stand out of order keeps that order across a save", "[presets][configuration]")
{
    const config::binding bound = binding_over(row(3u, six_vector(9.0)) + row(1u, six_vector(8.0)) + row(2u, six_vector(7.0)), "hand-ordered.xml");
    const supplied chosen       = a_chain(3);

    const manipulator::screw_modeling_window::edit_route spelling = presets::screw_table_edits(derived_three(), motions().frame);
    REQUIRE(spelling);
    REQUIRE(config::save(bound, spelling(carried(bound), at, chosen)).has_value());

    const std::vector<std::string> present = carried(bound).identities(keys_of_rows());
    REQUIRE(present.size() == 3u);
    CHECK(present == std::vector<std::string>{"3", "1", "2"});
    require_same_screws(opened(carried(bound), derived_three()), chosen);
}

// A chain somebody discarded is a chain the document has to stop carrying. The window hands back an
// entry per joint holding nothing, and a save writing no row for those is one that restores the
// discarded chain the next time the scenario is opened.
TEST_CASE("a chain whose joints were all discarded leaves the document carrying none of their rows", "[presets][configuration]")
{
    const config::binding bound = binding_over(rows_through(6u), "discarded.xml");
    REQUIRE(carried(bound).identities(keys_of_rows()).size() == 6u);

    supplied discarded;
    discarded.screws.resize(6u);
    REQUIRE(config::save(bound, written(carried(bound), derived_six(), discarded)).has_value());

    CHECK(carried(bound).identities(keys_of_rows()).empty());

    const supplied reopened = opened(carried(bound), derived_six());
    REQUIRE(reopened.screws.size() == 6u);
    for(std::size_t joint = 0u; joint < reopened.screws.size(); ++joint)
    {
        INFO("joint " << joint);
        CHECK_FALSE(reopened.screws[joint].has_value());
    }
}

// Taking a row out moves every row standing after it one place up, and the rows the same save writes
// into were addressed against the document as it stood before any of that.
TEST_CASE("a joint discarded beside two still supplied leaves those two carrying what was written", "[presets][configuration]")
{
    const config::binding bound = binding_over(rows_through(3u), "discarded-among-kept.xml");

    supplied chosen = a_chain(3);
    chosen.screws.front().reset();
    REQUIRE(config::save(bound, written(carried(bound), derived_three(), chosen)).has_value());

    CHECK(carried(bound).identities(keys_of_rows()) == std::vector<std::string>{"2", "3"});

    const supplied reopened = opened(carried(bound), derived_three());
    REQUIRE(reopened.screws.size() == 3u);
    CHECK_FALSE(reopened.screws.front().has_value());
    for(std::size_t joint = 1u; joint < reopened.screws.size(); ++joint)
    {
        INFO("joint " << joint);
        REQUIRE(reopened.screws[joint].has_value());
        CHECK((*reopened.screws[joint] - *chosen.screws[joint]).norm() == 0.0);
    }
}

TEST_CASE("a chain saved into a document whose lines end in a carriage return and a line feed takes a row out and leaves every line ending that way", "[presets][configuration]")
{
    const config::binding bound = binding_at("discarded-crlf.xml");
    REQUIRE(config::save(bound, written(carried(bound), derived_three(), a_chain(3))).has_value());
    ended_with_carriage_returns("discarded-crlf.xml");

    supplied chosen = a_chain(3);
    chosen.screws.front().reset();
    REQUIRE(config::save(bound, written(carried(bound), derived_three(), chosen)).has_value());

    const std::string after = bytes_at("discarded-crlf.xml");
    CHECK(occurrences(after, "\r\r") == 0u);
    CHECK(occurrences(after, "\r\n") == occurrences(after, "\n"));
    CHECK(carried(bound).identities(keys_of_rows()) == std::vector<std::string>{"2", "3"});

    const supplied reopened = opened(carried(bound), derived_three());
    REQUIRE(reopened.screws.size() == 3u);
    CHECK_FALSE(reopened.screws.front().has_value());
    for(std::size_t joint = 1u; joint < reopened.screws.size(); ++joint)
    {
        INFO("joint " << joint);
        REQUIRE(reopened.screws[joint].has_value());
        CHECK((*reopened.screws[joint] - *chosen.screws[joint]).norm() == 0.0);
    }
}

TEST_CASE("a row appended to a document whose lines end in a carriage return and a line feed ends its lines that way too", "[presets][configuration]")
{
    supplied first = a_chain(3);
    first.screws.front().reset();

    const config::binding bound = binding_at("appended-crlf.xml");
    const config::binding twin  = binding_at("appended-lf.xml");
    REQUIRE(config::save(bound, written(carried(bound), derived_three(), first)).has_value());
    REQUIRE(config::save(twin, written(carried(twin), derived_three(), first)).has_value());
    ended_with_carriage_returns("appended-crlf.xml");

    REQUIRE(config::save(bound, written(carried(bound), derived_three(), a_chain(3))).has_value());
    REQUIRE(config::save(twin, written(carried(twin), derived_three(), a_chain(3))).has_value());

    const std::string after = bytes_at("appended-crlf.xml");
    CHECK(occurrences(after, "\r\r") == 0u);
    CHECK(occurrences(after, "\r\n") == occurrences(after, "\n"));
    CHECK(with_line_feeds_only(after) == bytes_at("appended-lf.xml"));
    require_same_screws(opened(carried(bound), derived_three()), a_chain(3));
}

// A row the document carries is a row somebody wrote and a row it does not carry is a joint nobody
// supplied, so what makes a screw of all zeros a row is that somebody supplied it rather than that
// any leaf of it differs from the zero the declaration falls back to -- and none of them does.
TEST_CASE("a supplied screw every leaf of which reads as the fallback is a row of its own", "[presets][configuration]")
{
    const config::binding bound = binding_at("supplied-at-zero.xml");
    const screw_axis nothing    = screw_axis::Zero();

    supplied chosen;
    chosen.screws.assign(3u, nothing);
    REQUIRE(config::save(bound, written(carried(bound), derived_three(), chosen)).has_value());

    CHECK(carried(bound).identities(keys_of_rows()).size() == 3u);

    const supplied reopened = opened(carried(bound), derived_three());
    REQUIRE(reopened.screws.size() == 3u);
    for(std::size_t joint = 0u; joint < reopened.screws.size(); ++joint)
    {
        INFO("joint " << joint);
        REQUIRE(reopened.screws[joint].has_value());
        CHECK(reopened.screws[joint]->norm() == 0.0);
    }
}

// Whatever the document held before, what reads back after a save is the state the writer was
// handed: each entry holds a screw exactly where it held one, bit for bit, and nothing past its end.
TEST_CASE("a chain read back after a save is the chain that was saved, whatever the document held before", "[presets][configuration]")
{
    for(const shaped &document : document_shapes())
        for(const auto &[state_named, state] : state_shapes())
        {
            const std::string name = std::string("invariant-") + document.named + "-" + state_named + ".xml";
            INFO("a document " << document.named << " handed a state " << state_named);

            const config::binding bound = document.carried ? binding_over(document.body, name.c_str()) : binding_at(name.c_str());
            REQUIRE(config::save(bound, written(carried(bound), derived_six(), state)).has_value());

            const supplied read = opened(carried(bound), derived_six());
            for(std::size_t joint = 0u; joint < std::max(read.screws.size(), state.screws.size()); ++joint)
            {
                INFO("joint " << joint);
                const bool meant = holds(state, joint);
                const bool back  = holds(read, joint);
                CHECK(back == meant);
                if(meant && back)
                    CHECK((*read.screws[joint] - *state.screws[joint]).norm() == 0.0);
            }
        }
}

// The writer reaches exactly as far as the reader reads: a screw held at the furthest joint a
// document may name is written and read back, and one a joint further out is not.
TEST_CASE("a chain holding a screw at the furthest joint a document may name is saved and read back", "[presets][configuration]")
{
    const std::size_t furthest  = 6u + presets::screw_table_greatest_surplus;
    const config::binding bound = binding_at("writer-furthest.xml");

    supplied chosen;
    chosen.screws.resize(furthest);
    chosen.screws.back() = six_vector(2.0);
    REQUIRE(config::save(bound, written(carried(bound), derived_six(), chosen)).has_value());

    const supplied read = opened(carried(bound), derived_six());
    REQUIRE(read.screws.size() == furthest);
    REQUIRE(read.screws.back().has_value());
    CHECK((*read.screws.back() - six_vector(2.0)).norm() == 0.0);
}

TEST_CASE("a chain holding a screw a joint past the furthest a document may name is refused by name and writes nothing", "[presets][configuration]")
{
    const std::size_t furthest  = 6u + presets::screw_table_greatest_surplus;
    const config::binding bound = binding_over(rows_through(6u), "writer-beyond.xml");
    const std::string before    = bytes_at("writer-beyond.xml");

    supplied chosen = a_chain(6u);
    chosen.screws.resize(furthest + 1u);
    chosen.screws.back() = six_vector(2.0);

    const expected<std::vector<config::edit>, config::error> changes = presets::write_screw_table(carried(bound), at, derived_six(), chosen, motions().frame);
    REQUIRE_FALSE(changes.has_value());

    INFO(changes.error().message);
    CHECK(changes.error().message.find(std::to_string(furthest + 1u)) != std::string::npos);
    CHECK(changes.error().message.find(std::to_string(furthest)) != std::string::npos);

    presets::screw_table_route(bound, derived_six(), motions().frame)(at, chosen);
    CHECK(bytes_at("writer-beyond.xml") == before);
}

// A document the reader refuses is one the writer refuses in the same words, so the two never
// disagree about it and nothing is written beside the row that cannot be read.
TEST_CASE("a document carrying a row the reader refuses is refused in the reader's words and left as it was", "[presets][configuration]")
{
    const config::binding bound = binding_over(row("07", six_vector(1.0)), "writer-unreadable.xml");
    const std::string before    = bytes_at("writer-unreadable.xml");

    const expected<supplied, config::error> read                     = presets::read_screw_table(carried(bound), at, derived_six(), motions().screw, motions().frame);
    const expected<std::vector<config::edit>, config::error> changes = presets::write_screw_table(carried(bound), at, derived_six(), a_chain(6u), motions().frame);
    REQUIRE_FALSE(read.has_value());
    REQUIRE_FALSE(changes.has_value());
    CHECK(changes.error().message == read.error().message);

    presets::screw_table_route(bound, derived_six(), motions().frame)(at, a_chain(6u));
    CHECK(bytes_at("writer-unreadable.xml") == before);
}
