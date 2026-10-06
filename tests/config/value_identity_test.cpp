#include "read_back.h"

#include "praxis/config/error.h"
#include "praxis/config/writer.h"
#include "praxis/config/declaration.h"

#include "praxis/compat/expected.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cmath>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::config;

namespace {

struct pair_of
{
    field_kind kind;
    std::string one;
    std::string other;
};

void require_each(std::span<const pair_of> pairs, bool expected_one)
{
    for(const pair_of &pair : pairs)
    {
        INFO("'" << pair.one << "' against '" << pair.other << "'");
        CHECK(one_value(pair.kind, pair.one, pair.other) == expected_one);
        CHECK(one_value(pair.kind, pair.other, pair.one) == expected_one);
    }
}

void require_each_side_reads(std::span<const pair_of> pairs)
{
    for(const pair_of &pair : pairs)
    {
        if(pair.kind != field_kind::real)
            continue;

        INFO("'" << pair.one << "' against '" << pair.other << "'");
        REQUIRE(one_value(pair.kind, pair.one, pair.one));
        REQUIRE(one_value(pair.kind, pair.other, pair.other));
    }
}

std::filesystem::path carrying_zero()
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-config-value-identity";
    std::filesystem::create_directories(directory);

    const std::filesystem::path where = directory / "zero.xml";
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<probe>\n    <panel scale=\"0\" shown=\"false\"/>\n</probe>\n";
    return where;
}

expected<void, error> read_back(const std::string &scale, const std::string &shown)
{
    declaration shape("probe");
    shape.group("panel").field("panel/scale", field_kind::real, "1.0").field("panel/shown", field_kind::flag, "true");

    const std::array<std::string, 2> keys{"panel/scale", "panel/shown"};
    const std::array<std::string, 2> values{scale, shown};
    return reads_as_written(shape, carrying_zero(), keys, values, {});
}

}

TEST_CASE("a real is one value however it is spelled, and two reals however close stay two", "[config]")
{
    const std::array<pair_of, 9> one{{
            {field_kind::real, "0.10", "0.1"},
            {field_kind::real, "-0", "0"},
            {field_kind::real, "1", "1.0"},
            {field_kind::real, "-0.0", "0e5"},
            {field_kind::real, "1.5", " 1.5 "},
            {field_kind::real, "+1.5", "1.5"},
            {field_kind::real, "4.9e-324", exact_text(std::nextafter(0.0, 1.0))},
            {field_kind::real, ".5", "0.5"},
            {field_kind::real, "5.", "5"},
    }};
    const std::array<pair_of, 3> two{{
            {field_kind::real, "0.1", "0.2"},
            {field_kind::real, exact_text(0.1), exact_text(std::nextafter(0.1, 1.0))},
            {field_kind::real, "0", exact_text(std::nextafter(0.0, 1.0))},
    }};

    require_each_side_reads(one);
    require_each_side_reads(two);
    require_each(one, true);
    require_each(two, false);
}

TEST_CASE("a flag, an integer, a text and a choice are each compared as their own kind reads them", "[config]")
{
    const std::array<pair_of, 7> one{{
            {field_kind::flag, "1", "true"},
            {field_kind::flag, "0", "false"},
            {field_kind::integer, "07", "7"},
            {field_kind::text, "0.1", "0.1"},
            {field_kind::text, "", ""},
            {field_kind::choice, "fast", "fast"},
            {field_kind::text, " a ", " a "},
    }};
    const std::array<pair_of, 4> two{{
            {field_kind::flag, "1", "false"},
            {field_kind::integer, "7", "8"},
            {field_kind::text, "0.1", "0.10"},
            {field_kind::choice, "fast", " fast"},
    }};

    require_each(one, true);
    require_each(two, false);
}

TEST_CASE("a text that does not read as its kind is one value with nothing, itself included", "[config]")
{
    const std::array<pair_of, 12> none{{
            {field_kind::real, "abc", "abc"},
            {field_kind::real, "", ""},
            {field_kind::real, "abc", "0"},
            {field_kind::real, "1e-400", "1e-400"},
            {field_kind::real, "-1e-400", "-1e-400"},
            {field_kind::real, "2e-324", "2e-324"},
            {field_kind::real, "1e400", "1e400"},
            {field_kind::real, "0x1p3", "0x1p3"},
            {field_kind::real, "nan", "nan"},
            {field_kind::real, "inf", "inf"},
            {field_kind::flag, "yes", "yes"},
            {field_kind::integer, "7.0", "7"},
    }};

    require_each(none, false);
}

TEST_CASE("a read-back agrees with a value that reads back as one value however it was written, and refuses one that reads back as another", "[config]")
{
    CHECK(read_back("-0", "0").has_value());

    const expected<void, error> other = read_back("0.1", "false");
    REQUIRE_FALSE(other.has_value());
    CHECK(other.error().code == error_code::rejected_content);
    CHECK(other.error().message.find("panel/scale") != std::string::npos);
    CHECK(other.error().message.find("0.1") != std::string::npos);

    const std::string least = exact_text(std::nextafter(0.0, 1.0));
    REQUIRE(one_value(field_kind::real, least, least));
    const expected<void, error> neighbor = read_back(least, "false");
    REQUIRE_FALSE(neighbor.has_value());
    CHECK(neighbor.error().code == error_code::rejected_content);
    CHECK(neighbor.error().message.find("panel/scale") != std::string::npos);
}

TEST_CASE("a save spells a negative zero for a real leaf as 0 and keeps every other edit as offered", "[config]")
{
    declaration shape("probe");
    shape.group("panel").field("panel/scale", field_kind::real, "1.0").field("panel/label", field_kind::text, "");

    const std::vector<edit> offered{edit{"panel/scale", "-0"},  edit{"panel/scale", "-0.0"}, edit{"panel/scale", "-0e3"}, edit{"panel/scale", " -0 "},
                                    edit{"panel/scale", "0.0"}, edit{"panel/scale", "-1"},   edit{"panel/label", "-0"},   edit{"panel/scale", "-0", edit_kind::taken_out}};
    const std::array<std::string, 8> spelled{"0", "0", "0", "0", "0.0", "-1", "-0", "-0"};

    const std::vector<edit> written = as_written(shape, offered);
    REQUIRE(written.size() == offered.size());
    for(std::size_t which = 0; which < offered.size(); ++which)
    {
        INFO("'" << offered[which].key << "' offered as '" << offered[which].value << "'");
        CHECK(written[which].key == offered[which].key);
        CHECK(written[which].kind == offered[which].kind);
        CHECK(written[which].value == spelled[which]);
    }
}
