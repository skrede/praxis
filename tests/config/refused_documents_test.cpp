#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <regex>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::config;

namespace {

declaration described()
{
    declaration shape("probe");
    shape.group("window")
            .field("window/title", field_kind::text, "untitled")
            .field("window/width", field_kind::integer, "800")
            .field("window/scale", field_kind::real, "1.5")
            .choice("window/mode", {"fast", "careful"}, "fast")
            .group("stations")
            .collection("stations/station", "name");
    return shape;
}

std::filesystem::path written(const std::string &name, std::string_view body)
{
    const std::filesystem::path directory = std::filesystem::weakly_canonical(std::filesystem::temp_directory_path()) / "praxis-config-refused";
    std::filesystem::create_directories(directory);
    const std::filesystem::path where = directory / name;

    std::ofstream out(where, std::ios::trunc | std::ios::binary);
    out << body;
    return where;
}

std::string text_of(const std::filesystem::path &where)
{
    std::ifstream in(where, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

expected<document, error> loaded(const std::string &name, std::string_view body)
{
    const std::filesystem::path where = written(name, body);
    return load(described(), resolve(where, where.parent_path()));
}

std::vector<std::string_view> lines_of(std::string_view message)
{
    std::vector<std::string_view> lines;
    for(std::size_t from = 0; from <= message.size(); from += lines.back().size() + 1)
        lines.push_back(message.substr(from, message.find('\n', from) - from));
    return lines;
}

// Whether one line of `message` names both the path and the value a finding is about.
bool names(std::string_view message, std::string_view path, std::string_view value)
{
    return std::ranges::any_of(lines_of(message), [&](std::string_view line) { return line.find(path) != std::string_view::npos && line.find(value) != std::string_view::npos; });
}

// How often `text` stands on the lines of `message` that name `path`.
std::size_t times_named(std::string_view message, std::string_view path, std::string_view text)
{
    std::size_t times = 0;
    for(const std::string_view line : lines_of(message))
        if(line.find(path) != std::string_view::npos)
            for(std::size_t at = line.find(text); at != std::string_view::npos; at = line.find(text, at + 1))
                ++times;
    return times;
}

}

TEST_CASE("a refused document names every finding it carries in one message", "[config]")
{
    const std::filesystem::path where    = written("three-findings.xml", "<probe><window mode=\"x\" width=\"3.5\" scale=\"y\"/></probe>\n");
    const expected<document, error> read = load(described(), resolve(where, where.parent_path()));

    REQUIRE_FALSE(read.has_value());
    CHECK(read.error().code == error_code::rejected_content);
    CHECK(read.error().message.find(where.string()) != std::string::npos);
    CHECK(names(read.error().message, "window/mode", "x"));
    CHECK(names(read.error().message, "window/width", "3.5"));
    CHECK(names(read.error().message, "window/scale", "y"));
}

TEST_CASE("a text where a collection or the whole document is declared is malformed, not a mismatched space", "[config]")
{
    const expected<document, error> instance = loaded("instance-text.xml", "<probe><stations><station>oops</station></stations></probe>\n");
    const expected<document, error> root     = loaded("root-text.xml", "<probe>oops</probe>\n");

    REQUIRE_FALSE(instance.has_value());
    CHECK(instance.error().code == error_code::malformed_source);
    REQUIRE_FALSE(root.has_value());
    CHECK(root.error().code == error_code::malformed_source);
}

TEST_CASE("a root that is not the declared space is a mismatched space whatever it carries", "[config]")
{
    const expected<document, error> mixed = loaded("other-mixed.xml", "<other><window title=\"a\">text</window></other>\n");
    const expected<document, error> plain = loaded("other-plain.xml", "<other><window title=\"a\"/></other>\n");

    REQUIRE_FALSE(mixed.has_value());
    CHECK(mixed.error().code == error_code::mismatched_space);
    REQUIRE_FALSE(plain.has_value());
    CHECK(plain.error().code == error_code::mismatched_space);
}

TEST_CASE("a choice outside its list is named with the allowed value nearest to it", "[config]")
{
    const expected<document, error> read = loaded("misspelled-choice.xml", "<probe><window mode=\"fsat\"/></probe>\n");

    REQUIRE_FALSE(read.has_value());
    CHECK(read.error().code == error_code::rejected_content);
    CHECK(names(read.error().message, "window/mode", "fsat"));
    CHECK(times_named(read.error().message, "window/mode", "'fast'") == 2);
    CHECK(times_named(read.error().message, "window/mode", "'careful'") == 1);
}

TEST_CASE("a document that does not parse is refused with the line and column of the fault", "[config]")
{
    const expected<document, error> read = loaded("unclosed.xml", "<probe>\n  <window title=\"a\">\n</probe>\n");

    REQUIRE_FALSE(read.has_value());
    CHECK(read.error().code == error_code::malformed_source);
    CHECK(std::regex_search(read.error().message, std::regex("line 3\\b")));
    CHECK(std::regex_search(read.error().message, std::regex("column 3\\b")));
}

TEST_CASE("the nearest allowed value is the one fewest weighted edits away, ties going to the lexicographically first", "[config]")
{
    declaration shape("probe");
    shape.group("dial").choice("dial/grade", {"b2", "by", "bx", "a1", "ab"}, "ab");
    const std::filesystem::path within = written("one-class-apart.xml", "<probe><dial grade=\"ac\"/></probe>\n");
    const std::filesystem::path tied   = written("two-equally-near.xml", "<probe><dial grade=\"bz\"/></probe>\n");

    const expected<document, error> substituted = load(shape, resolve(within, within.parent_path()));
    const expected<document, error> tie         = load(shape, resolve(tied, tied.parent_path()));

    REQUIRE_FALSE(substituted.has_value());
    CHECK(times_named(substituted.error().message, "dial/grade", "'ab'") == 2);
    REQUIRE_FALSE(tie.has_value());
    CHECK(times_named(tie.error().message, "dial/grade", "'bx'") == 2);
}

TEST_CASE("a leaf written twice is malformed by name through load and save, and a group written twice is not", "[config]")
{
    constexpr std::array twice{std::string_view("<probe><window><title>one</title><title>two</title></window></probe>\n"),
                               std::string_view("<probe><window title=\"attr\"><title>elem</title></window></probe>\n")};
    for(const std::string_view body : twice)
    {
        INFO(body);
        const std::filesystem::path where    = written("leaf-twice.xml", body);
        const expected<document, error> read = load(described(), resolve(where, where.parent_path()));
        const expected<void, error> saved    = save(described(), resolve(where, where.parent_path()), std::vector<edit>{edit{"window/title", "new"}});

        REQUIRE_FALSE(read.has_value());
        CHECK(read.error().code == error_code::malformed_source);
        CHECK(read.error().message.find("'window/title' is written twice") != std::string::npos);
        REQUIRE_FALSE(saved.has_value());
        CHECK(saved.error().code == error_code::malformed_source);
        CHECK(saved.error().message.find("'window/title' is written twice") != std::string::npos);
        CHECK(text_of(where) == body);
    }

    const expected<document, error> group = loaded("group-twice.xml", "<probe><window title=\"a\"/><window width=\"3\"/></probe>\n");
    const expected<document, error> empty = loaded("empty-then-full.xml", "<probe><window/><window title=\"b\"/></probe>\n");
    REQUIRE(group.has_value());
    CHECK(group.value().text("window/title").value() == "a");
    CHECK(group.value().integer("window/width").value() == 3);
    REQUIRE(empty.has_value());
    CHECK(empty.value().text("window/title").value() == "b");
}

TEST_CASE("a declared leaf carrying a child element is malformed by name at load, no save writes into it, and an attribute on it is not", "[config]")
{
    struct refusal
    {
        std::string_view body;
        error_code saved;
        std::string_view says;
    };
    constexpr std::string_view finding  = "'window/title' carries the element 'x' where only its value can stand";
    constexpr std::string_view no_place = "has no place for window/title";
    constexpr std::array beside{refusal{"<probe><window><title>ab<x/>cd</title></window></probe>\n", error_code::malformed_source, finding},
                                refusal{"<probe><window><title><x/>cd</title></window></probe>\n", error_code::unlocatable_key, no_place},
                                refusal{"<probe><window><title><x/></title></window></probe>\n", error_code::unlocatable_key, no_place}};
    for(const refusal &one : beside)
    {
        INFO(one.body);
        const std::filesystem::path where    = written("leaf-with-child.xml", one.body);
        const expected<document, error> read = load(described(), resolve(where, where.parent_path()));
        const expected<void, error> saved    = save(described(), resolve(where, where.parent_path()), std::vector<edit>{edit{"window/title", "new"}});

        REQUIRE_FALSE(read.has_value());
        CHECK(read.error().code == error_code::malformed_source);
        CHECK(read.error().message.find(finding) != std::string::npos);
        REQUIRE_FALSE(saved.has_value());
        CHECK(saved.error().code == one.saved);
        CHECK(saved.error().message.find(one.says) != std::string::npos);
        CHECK(text_of(where) == one.body);
    }

    const expected<document, error> identity  = loaded("identity-with-child.xml", "<probe><stations><station><name>a<x/></name></station></stations></probe>\n");
    const expected<document, error> attribute = loaded("leaf-with-attribute.xml", "<probe><window><title lang=\"en\">abc</title></window></probe>\n");
    REQUIRE_FALSE(identity.has_value());
    CHECK(identity.error().code == error_code::malformed_source);
    CHECK(identity.error().message.find("'stations/station[0]/name' carries the element 'x' where only its value can stand") != std::string::npos);
    REQUIRE(attribute.has_value());
    CHECK(attribute.value().text("window/title").value() == "abc");
    CHECK(attribute.value().origin_of("window/title").kind == origin_kind::source);
}

TEST_CASE("an attribute written twice or a text beside attributes on a declared path refuses the document by name", "[config]")
{
    const expected<document, error> repeated = loaded("repeated-attribute.xml", "<probe><window title=\"a\" title=\"b\"/></probe>\n");
    const expected<document, error> mixed    = loaded("text-beside-attributes.xml", "<probe><window title=\"a\">text</window></probe>\n");

    REQUIRE_FALSE(repeated.has_value());
    CHECK(repeated.error().code == error_code::malformed_source);
    CHECK(repeated.error().message.find("'window/title' is written twice on one element") != std::string::npos);
    REQUIRE_FALSE(mixed.has_value());
    CHECK(mixed.error().code == error_code::malformed_source);
    CHECK(mixed.error().message.find("'window' carries the text 'text' beside attributes") != std::string::npos);
}

TEST_CASE("a document refused for a value also names each undeclared path with the declared path nearest it", "[config]")
{
    const expected<document, error> content    = loaded("value-and-undeclared.xml", "<probe><window width=\"3.5\" titel=\"b\"/></probe>\n");
    const expected<document, error> structural = loaded("twice-and-undeclared.xml", "<probe><window title=\"a\" title=\"b\" titel=\"c\"/></probe>\n");
    const expected<document, error> value_only = loaded("value-only.xml", "<probe><window width=\"3.5\"/></probe>\n");

    REQUIRE_FALSE(content.has_value());
    CHECK(content.error().code == error_code::rejected_content);
    CHECK(names(content.error().message, "window/width", "3.5"));
    CHECK(names(content.error().message, "'window/titel'", "'window/title'"));
    REQUIRE_FALSE(structural.has_value());
    CHECK(structural.error().code == error_code::malformed_source);
    CHECK(structural.error().message.find("'window/titel'") != std::string::npos);
    REQUIRE_FALSE(value_only.has_value());
    CHECK(lines_of(value_only.error().message).size() == 2);
}
