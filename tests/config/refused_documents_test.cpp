#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <cstddef>
#include <fstream>
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

// Whether one line of `message` names both the path and the value a finding is about.
bool names(const std::string &message, std::string_view path, std::string_view value)
{
    std::size_t from = 0;
    while(from <= message.size())
    {
        const std::size_t ends      = message.find('\n', from);
        const std::string_view line = std::string_view(message).substr(from, ends == std::string::npos ? std::string::npos : ends - from);
        if(line.find(path) != std::string_view::npos && line.find(value) != std::string_view::npos)
            return true;
        if(ends == std::string::npos)
            return false;
        from = ends + 1;
    }
    return false;
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
