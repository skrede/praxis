#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <catch2/catch_test_macros.hpp>

#include <regex>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
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
