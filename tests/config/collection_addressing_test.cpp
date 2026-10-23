#include "../presets/scratch_directory.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::config;

namespace {

constexpr std::string_view two_stations = "<probe>\n"
                                          "    <stations>\n"
                                          "        <station name=\"A\"><slot id=\"p\" v=\"1\"/></station>\n"
                                          "        <station name=\"B\"><slot id=\"q\" v=\"2\"/></station>\n"
                                          "    </stations>\n"
                                          "</probe>\n";

declaration nested()
{
    declaration shape("probe");
    shape.group("stations").collection("stations/station", "name").collection("stations/station/slot", "id").field("stations/station/slot/v", field_kind::integer, "0");
    return shape;
}

declaration paneled()
{
    declaration shape("probe");
    shape.group("g").collection("g/c", "name").field("g/c/v", field_kind::integer, "0");
    shape.collection("station", "name").group("station/panel").field("station/panel/scale", field_kind::real, "1.0");
    return shape;
}

declaration counted()
{
    declaration shape("probe");
    shape.collection("station", "name").field("station/width", field_kind::integer, "0");
    return shape;
}

std::filesystem::path written(const std::string &name, std::string_view body)
{
    const std::filesystem::path directory = fixture::shared_scratch_directory() / "config_collection_addressing";
    std::filesystem::create_directories(directory);
    const std::filesystem::path where = directory / name;
    std::ofstream(where, std::ios::binary | std::ios::trunc) << body;
    return where;
}

location at(const std::filesystem::path &where)
{
    return resolve(where, where.parent_path());
}

std::string why(const expected<void, error> &outcome)
{
    return outcome.has_value() ? std::string() : outcome.error().message;
}

std::string text_of(const std::filesystem::path &where)
{
    std::ifstream in(where, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

}

TEST_CASE("the identities of a nested collection are those under the outer instance its path addresses", "[config]")
{
    const std::filesystem::path where    = written("nested-identities.xml", two_stations);
    const expected<document, error> read = load(nested(), at(where));
    REQUIRE(read.has_value());

    CHECK(read.value().identities("stations/station[1]/slot") == std::vector<std::string>{"q"});
    CHECK(read.value().identities("stations/station[0]/slot") == std::vector<std::string>{"p"});
    CHECK(read.value().identities("stations/station/slot") == std::vector<std::string>{"p"});
    CHECK(read.value().identities("stations/station[0]").empty());

    const expected<std::string, error> keyed = read.value().key("stations/station[1]/slot", "q", "v");
    REQUIRE(keyed.has_value());
    CHECK(keyed.value() == "stations/station[1]/slot[0]/v");
    CHECK(read.value().integer(keyed.value()).value() == 2);
}

TEST_CASE("an identity is created and renamed under the second outer instance and reads back", "[config]")
{
    const std::filesystem::path created = written("nested-created.xml", two_stations);
    const std::vector<edit> grown{edit{"stations/station[1]/slot[1]/id", "z"}, edit{"stations/station[1]/slot[1]/v", "7"}};
    const expected<void, error> saved_grown = save(nested(), at(created), grown);
    INFO(why(saved_grown));
    REQUIRE(saved_grown.has_value());
    const expected<document, error> after_growth = load(nested(), at(created));
    REQUIRE(after_growth.has_value());
    CHECK(after_growth.value().identities("stations/station[1]/slot") == std::vector<std::string>{"q", "z"});
    CHECK(after_growth.value().identities("stations/station[0]/slot") == std::vector<std::string>{"p"});

    const std::filesystem::path renamed = written("nested-renamed.xml", two_stations);
    const std::vector<edit> rename{edit{"stations/station[1]/slot[0]/id", "r"}};
    const expected<void, error> saved_rename = save(nested(), at(renamed), rename);
    INFO(why(saved_rename));
    REQUIRE(saved_rename.has_value());
    const expected<document, error> after_rename = load(nested(), at(renamed));
    REQUIRE(after_rename.has_value());
    CHECK(after_rename.value().identities("stations/station[1]/slot") == std::vector<std::string>{"r"});
    CHECK(after_rename.value().identities("stations/station[0]/slot") == std::vector<std::string>{"p"});
}

TEST_CASE("a read passing a collection without naming an instance counts every instance the collection holds", "[config]")
{
    const std::filesystem::path where    = written("crossing-count.xml", "<probe><station name=\"alpha\" width=\"10\"/><station/><station name=\"beta\" width=\"90\"/></probe>\n");
    const expected<document, error> read = load(counted(), at(where));
    REQUIRE(read.has_value());
    REQUIRE(read.value().identities("station").size() == 3u);

    const expected<std::int64_t, error> crossed = read.value().integer("station/width");
    REQUIRE_FALSE(crossed.has_value());
    CHECK(crossed.error().code == error_code::instance_required);
    CHECK(crossed.error().message.find("one of its 3 instances") != std::string::npos);
}

TEST_CASE("a collection instance inside a second copy of its group is refused with the group by name at load, and no save writes into it", "[config]")
{
    constexpr std::string_view body      = "<probe><g><c name=\"a\" v=\"1\"/></g><g><c name=\"b\" v=\"2\"/></g></probe>\n";
    const std::filesystem::path where    = written("instance-in-second-copy.xml", body);
    const expected<document, error> read = load(paneled(), at(where));
    const expected<void, error> saved    = save(paneled(), at(where), std::vector<edit>{edit{"g/c[1]/v", "9"}});

    REQUIRE_FALSE(read.has_value());
    CHECK(read.error().code == error_code::malformed_source);
    CHECK(read.error().message.find("'g' is written twice") != std::string::npos);
    CHECK_FALSE(saved.has_value());
    CHECK(text_of(where) == body);
}

TEST_CASE("one group under each of two instances is written once, and a group repeated inside one instance is not", "[config]")
{
    const std::filesystem::path apart =
            written("group-per-instance.xml", "<probe><station name=\"a\"><panel scale=\"2\"/></station><station name=\"b\"><panel scale=\"3\"/></station></probe>\n");
    const std::filesystem::path together = written("group-twice-in-instance.xml", "<probe><station name=\"a\"><panel scale=\"2\"/><panel/></station></probe>\n");

    const expected<document, error> each = load(paneled(), at(apart));
    REQUIRE(each.has_value());
    CHECK(each.value().real("station[1]/panel/scale").value() == 3.0);

    const expected<document, error> twice = load(paneled(), at(together));
    REQUIRE_FALSE(twice.has_value());
    CHECK(twice.error().message.find("'station[0]/panel' is written twice") != std::string::npos);
}
