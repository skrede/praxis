#include "captured_log.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <optional>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::config;

namespace {

constexpr std::array<std::string_view, 2> under_root_and_group{"station", "stations/station"};

constexpr std::string_view gapped = "<station name=\"alpha\" width=\"10\"/><station/><station name=\"beta\" width=\"90\"/>";

bool grouped(std::string_view collection)
{
    return collection != "station";
}

declaration described(std::string_view collection)
{
    declaration shape("probe");
    if(grouped(collection))
        shape.group("stations");
    shape.collection(std::string(collection), "name").field(std::string(collection) + "/width", field_kind::integer, "0");
    return shape;
}

std::string within(std::string_view collection, std::string_view instances)
{
    return grouped(collection) ? "<probe><stations>" + std::string(instances) + "</stations></probe>\n" : "<probe>" + std::string(instances) + "</probe>\n";
}

std::filesystem::path written(std::string_view name, std::string_view collection, std::string_view body)
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-config-instance-positions";
    std::filesystem::create_directories(directory);
    const std::filesystem::path where = directory / (std::string(name) + (grouped(collection) ? "-group.xml" : "-root.xml"));
    std::ofstream(where, std::ios::binary | std::ios::trunc) << body;
    return where;
}

location at(const std::filesystem::path &where)
{
    return resolve(where, where.parent_path());
}

std::string text_of(const std::filesystem::path &where)
{
    std::ifstream in(where, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

std::string replaced(std::string text, std::string_view from, std::string_view to)
{
    const std::size_t found = text.find(from);
    REQUIRE(found != std::string::npos);
    return text.replace(found, from.size(), to);
}

document loaded(const declaration &shape, const std::filesystem::path &where)
{
    expected<document, error> read = load(shape, at(where));
    INFO((read ? std::string() : read.error().message));
    REQUIRE(read.has_value());
    return read.value();
}

bool unlocatable(const expected<std::string, error> &addressed)
{
    return !addressed.has_value() && addressed.error().code == error_code::unlocatable_key;
}

void saved(const declaration &shape, const std::filesystem::path &where, const std::vector<edit> &changes)
{
    const expected<void, error> written_back = save(shape, at(where), changes);
    INFO((written_back ? std::string() : written_back.error().message));
    REQUIRE(written_back.has_value());
}

void renames_beta_past_an_empty_instance(std::string_view collection)
{
    const declaration shape           = described(collection);
    const std::string original        = within(collection, gapped);
    const std::filesystem::path where = written("renamed", collection, original);

    const document read = loaded(shape, where);
    REQUIRE(read.identities(collection) == std::vector<std::string>{"alpha", "", "beta"});
    const std::string named = read.key(collection, "beta", "name").value();
    REQUIRE(named == std::string(collection) + "[2]/name");

    saved(shape, where, {edit{named, "gamma"}});
    REQUIRE(text_of(where) == replaced(original, "name=\"beta\"", "name=\"gamma\""));
    const document reread = loaded(shape, where);
    REQUIRE(reread.identities(collection) == std::vector<std::string>{"alpha", "", "gamma"});
    REQUIRE(reread.integer(reread.key(collection, "gamma", "width").value()).value() == 90);
}

void widens_through_its_key(std::string_view collection, const std::vector<std::string> &identities, std::size_t position)
{
    INFO(std::string(collection) + " " + identities[position]);
    const declaration shape           = described(collection);
    const std::string original        = within(collection, gapped);
    const std::filesystem::path where = written("handed-out", collection, original);
    const document read               = loaded(shape, where);
    const std::string addressed       = read.key(collection, identities[position], "width").value();
    REQUIRE(addressed == std::string(collection) + "[" + std::to_string(position) + "]/width");

    const std::string width   = std::to_string(read.integer(addressed).value());
    const std::string widened = width + "000";
    saved(shape, where, {edit{addressed, widened}});
    const std::string named = "name=\"" + identities[position] + "\" width=\"";
    REQUIRE(text_of(where) == replaced(original, named + width + "\"", named + widened + "\""));
    const document reread = loaded(shape, where);
    REQUIRE(std::to_string(reread.integer(addressed).value()) == widened);
    for(const std::string &other : identities)
        if(!other.empty() && other != identities[position])
            REQUIRE(reread.integer(reread.key(collection, other, "width").value()).value() == read.integer(read.key(collection, other, "width").value()).value());
}

}

TEST_CASE("a save through the key of an instance after one carrying nothing lands in that instance, under the root", "[config]")
{
    renames_beta_past_an_empty_instance("station");
}

TEST_CASE("a save through the key of an instance after one carrying nothing lands in that instance, under a group", "[config]")
{
    renames_beta_past_an_empty_instance("stations/station");
}

TEST_CASE("an instance carrying only an undeclared element keeps its place for a save beside it and an append past it", "[config]")
{
    const std::string_view collection = "stations/station";
    const declaration shape           = described(collection);
    const std::string original        = within(collection, "<station><nmae>x</nmae></station><station name=\"beta\" width=\"90\"/>");
    const std::filesystem::path where = written("stray", collection, original);

    const document read = loaded(shape, where);
    REQUIRE(read.identities(collection) == std::vector<std::string>{"", "beta"});
    const std::string addressed = read.key(collection, "beta", "width").value();
    REQUIRE(addressed == "stations/station[1]/width");
    saved(shape, where, {edit{addressed, "7"}});
    const std::string beside = replaced(original, "width=\"90\"", "width=\"7\"");
    REQUIRE(text_of(where) == beside);

    const std::string appended = "stations/station[" + std::to_string(loaded(shape, where).identities(collection).size()) + "]";
    saved(shape, where, {edit{appended + "/name", "gamma"}, edit{appended + "/width", "3"}});
    REQUIRE(text_of(where).starts_with(beside.substr(0, beside.find("</stations>"))));
    const document reread = loaded(shape, where);
    REQUIRE(reread.identities(collection) == std::vector<std::string>{"", "beta", "gamma"});
    REQUIRE(reread.integer(reread.key(collection, "gamma", "width").value()).value() == 3);
    REQUIRE(reread.integer(reread.key(collection, "beta", "width").value()).value() == 7);
}

TEST_CASE("every key a document hands out addresses the element it names, under the root and under a group", "[config]")
{
    for(const std::string_view collection : under_root_and_group)
    {
        const std::vector<std::string> identities = loaded(described(collection), written("handed-out", collection, within(collection, gapped))).identities(collection);
        for(std::size_t position = 0; position < identities.size(); ++position)
            if(!identities[position].empty())
                widens_through_its_key(collection, identities, position);
    }
}

TEST_CASE("an empty identity names no instance", "[config]")
{
    const declaration shape    = described("station");
    const document gapped_read = loaded(shape, written("empty-identity", "station", within("station", gapped)));
    REQUIRE(unlocatable(gapped_read.key("station", "", "width")));

    const document spelled_empty = loaded(shape, written("empty-identity-value", "station", within("station", "<station name=\"\" width=\"5\"/>")));
    REQUIRE(spelled_empty.identities("station") == std::vector<std::string>{""});
    REQUIRE(unlocatable(spelled_empty.key("station", "", "width")));
}

TEST_CASE("a collection whose instances carry nothing answers its leaves from the fallback", "[config]")
{
    for(const std::string_view collection : under_root_and_group)
    {
        const document read       = loaded(described(collection), written("carrying-nothing", collection, within(collection, "<station/>")));
        const std::string unnamed = std::string(collection) + "/width";
        REQUIRE(read.identities(collection) == std::vector<std::string>{""});
        REQUIRE(read.origin_of(unnamed).kind == origin_kind::fallback);
        REQUIRE(read.integer(unnamed).value() == 0);
    }
}

TEST_CASE("an undeclared-path warning names the ordinal the document's keys use", "[config]")
{
    const std::string_view collection = "stations/station";
    const std::filesystem::path where = written("warned", collection, within(collection, "<station><nme>a</nme></station><station name=\"b\" width=\"4\"/>"));

    std::optional<outcome> answered;
    const std::string said = tests::reported_by([&] { answered = load_or_defaults(described(collection), at(where), expectation::partial); });

    REQUIRE(answered.has_value());
    REQUIRE_FALSE(answered->failure.has_value());
    REQUIRE(said.find("'stations/station[0]/nme'") != std::string::npos);
    const expected<std::string, error> addressed = answered->values.key(collection, "b", "width");
    REQUIRE(addressed.value() == "stations/station[1]/width");
    REQUIRE(answered->values.integer(addressed.value()).value() == 4);
}
