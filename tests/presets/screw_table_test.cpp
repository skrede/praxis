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
#include <optional>
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

// The two states a row nobody supplied opens at, written out here so the case pins the values
// rather than asking the same function the reader asked.
screw_axis turning_at_the_origin()
{
    return (screw_axis() << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0).finished();
}

screw_axis translating_along_z()
{
    return (screw_axis() << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();
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

std::string row(std::size_t index, const std::optional<screw_axis> &screw)
{
    std::string element = "<joint index=\"" + std::to_string(index) + "\">";
    if(screw)
        element += triple("angular", screw->head<3>()) + triple("linear", screw->tail<3>());

    return element + "</joint>";
}

supplied opened(const config::document &values, const manipulator::screw_chain &derived)
{
    const expected<supplied, config::error> read = presets::read_screw_table(values, at, derived, motions().screw, motions().frame);
    INFO((read ? std::string() : read.error().message));
    REQUIRE(read.has_value());

    return read.value();
}

void require_same_screws(const supplied &read, const supplied &chosen)
{
    REQUIRE(read.screws.size() == chosen.screws.size());
    for(std::size_t joint = 0u; joint < chosen.screws.size(); ++joint)
    {
        INFO("joint " << joint);
        REQUIRE((read.screws[joint] - chosen.screws[joint]).norm() == 0.0);
    }
}

}

TEST_CASE("a chain written into a document reads back as the chain it was", "[presets][configuration]")
{
    const config::binding bound = binding_at("round-trip.xml");
    const supplied chosen       = a_chain(6);

    const std::vector<config::edit> changes = presets::write_screw_table(carried(bound), at, chosen, motions().frame);
    REQUIRE(changes.size() == 6u + 6u * 7u);
    REQUIRE(config::save(bound, changes).has_value());

    const supplied read = opened(carried(bound), derived_six());
    REQUIRE((read.home - chosen.home).norm() < 1.0e-5);
    require_same_screws(read, chosen);
}

TEST_CASE("a chain no document names opens every row at the degenerate screw and its home at the identity", "[presets][configuration]")
{
    const config::outcome absent = config::load_or_defaults(binding_at("no-chain-was-kept.xml"));
    REQUIRE(absent.failure.has_value());

    const supplied read = opened(absent.values, derived_six());
    REQUIRE((read.home - transform::Identity()).norm() == 0.0);
    REQUIRE(read.screws.size() == 6u);
    for(std::size_t joint = 0u; joint + 1u < read.screws.size(); ++joint)
    {
        INFO("joint " << joint);
        REQUIRE(read.screws[joint] == turning_at_the_origin());
    }
    REQUIRE(read.screws.back() == translating_along_z());
}

TEST_CASE("a document carrying only some of the chain's rows opens the rest at the degenerate screw", "[presets][configuration]")
{
    std::string body;
    for(std::size_t joint = 0u; joint < 3u; ++joint)
        body += row(joint + 1u, six_vector(1.0 + static_cast<double>(joint)));

    const supplied read = opened(authored(body, "some-rows.xml"), derived_six());
    REQUIRE(read.screws.size() == 6u);
    for(std::size_t joint = 0u; joint < 3u; ++joint)
    {
        INFO("joint " << joint);
        REQUIRE((read.screws[joint] - six_vector(1.0 + static_cast<double>(joint))).norm() < 1.0e-5);
    }
    for(std::size_t joint = 3u; joint < 5u; ++joint)
        REQUIRE(read.screws[joint] == turning_at_the_origin());
    REQUIRE(read.screws.back() == translating_along_z());
}

// A row somebody kept for a longer machine names a joint this one does not have, and no reading of
// it is a reading of this chain: the whole table is turned away rather than the surplus dropped.
TEST_CASE("a table naming a joint the machine's chain does not have is refused with both counts", "[presets][configuration]")
{
    std::string body;
    for(std::size_t joint = 0u; joint < 8u; ++joint)
        body += row(joint + 1u, six_vector(1.0 + static_cast<double>(joint)));

    const expected<supplied, config::error> read = presets::read_screw_table(authored(body, "wrong-length.xml"), at, derived_six(), motions().screw, motions().frame);
    REQUIRE_FALSE(read.has_value());

    INFO(read.error().message);
    REQUIRE(read.error().message.find("names 8 joints") != std::string::npos);
    REQUIRE(read.error().message.find("has 6") != std::string::npos);
}

TEST_CASE("a table naming a joint out of the chain's order is refused although it carries no more rows than the chain", "[presets][configuration]")
{
    const expected<supplied, config::error> read =
            presets::read_screw_table(authored(row(1u, six_vector(1.0)) + row(7u, six_vector(2.0)), "out-of-order.xml"), at, derived_six(), motions().screw, motions().frame);

    REQUIRE_FALSE(read.has_value());
}

TEST_CASE("a chain written twice with nothing moved between offers no second edit", "[presets][configuration]")
{
    const config::binding bound = binding_at("nothing-moved.xml");
    const supplied chosen       = a_chain(6);

    REQUIRE(config::save(bound, presets::write_screw_table(carried(bound), at, chosen, motions().frame)).has_value());
    REQUIRE(presets::write_screw_table(carried(bound), at, chosen, motions().frame).empty());
}

TEST_CASE("a row the document carries no instance of is appended at the collection's current size, named before it is filled", "[presets][configuration]")
{
    const config::document values = authored(row(2u, six_vector(2.5)) + row(3u, six_vector(3.5)), "appended.xml");
    const std::string rows        = std::string(at) + "/joint";

    const std::vector<config::edit> changes = presets::write_screw_table(values, at, a_chain(3), motions().frame);
    REQUIRE(changes.size() == 6u + 7u + 6u + 6u);

    REQUIRE(changes[6].key == rows + "[2]/index");
    REQUIRE(changes[6].value == "1");
    REQUIRE(changes[7].key == rows + "[2]/angular/x");
    REQUIRE(changes[13].key == rows + "[0]/angular/x");
    REQUIRE(changes[19].key == rows + "[1]/angular/x");
}

TEST_CASE("a route writes the chain it is handed into the binding's document, and a binding naming no document answers none", "[presets][configuration]")
{
    REQUIRE_FALSE(presets::screw_table_route(std::nullopt, motions().frame));
    REQUIRE_FALSE(presets::screw_table_route(config::binding{presets::screw_table_keyspace(), config::location{}, config::expectation::partial}, motions().frame));

    const config::binding bound                               = binding_at("routed.xml");
    const manipulator::screw_modeling_window::save_route keep = presets::screw_table_route(bound, motions().frame);
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
    const manipulator::screw_modeling_window::save_route keep = presets::screw_table_route(bound, motions().frame);
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

    REQUIRE(config::save(bound, presets::write_screw_table(carried(bound), at, chosen, motions().frame)).has_value());

    const manipulator::screw_modeling_window::edit_route spelling = presets::screw_table_edits(motions().frame);
    REQUIRE(spelling);
    CHECK(spelling(carried(bound), at, chosen).empty());
}

// A document somebody wrote by hand names its joints in whatever order it likes, and the reader
// resolves a row by the identity it carries; saving over it must not renumber those rows.
TEST_CASE("a table whose rows stand out of order keeps that order across a save", "[presets][configuration]")
{
    const config::binding bound = binding_over(row(3u, six_vector(9.0)) + row(1u, six_vector(8.0)) + row(2u, six_vector(7.0)), "hand-ordered.xml");
    const supplied chosen       = a_chain(3);

    const manipulator::screw_modeling_window::edit_route spelling = presets::screw_table_edits(motions().frame);
    REQUIRE(spelling);
    REQUIRE(config::save(bound, spelling(carried(bound), at, chosen)).has_value());

    const std::vector<std::string> present = carried(bound).identities(keys_of_rows());
    REQUIRE(present.size() == 3u);
    CHECK(present == std::vector<std::string>{"3", "1", "2"});
    require_same_screws(opened(carried(bound), derived_three()), chosen);
}
