#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>
#include <limits>
#include <locale>
#include <fstream>
#include <sstream>
#include <iomanip>
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
            .group("stations")
            .collection("stations/station", "name")
            .group("stations/station/panel")
            .field("stations/station/panel/width", field_kind::integer, "0")
            .field("stations/station/panel/scale", field_kind::real, "1.0");
    return shape;
}

expected<document, error> folded(const std::string &name, std::string_view body)
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-config-scoped";
    std::filesystem::create_directories(directory);
    const std::filesystem::path where = directory / name;

    std::ofstream out(where, std::ios::trunc);
    out << "<probe><window title=\"a\"/><stations>" << body << "</stations></probe>\n";
    out.close();

    return load(described(), resolve(where, directory));
}

std::string why(const expected<document, error> &outcome)
{
    return outcome.has_value() ? std::string() : outcome.error().message;
}

std::string exactly(double value)
{
    std::ostringstream printed;
    printed.imbue(std::locale::classic());
    printed << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    return printed.str();
}

}

TEST_CASE("a collection with no instance carries no identity and no scoped value", "[config]")
{
    const expected<document, error> present_document = folded("zero.xml", "");
    INFO(why(present_document));
    REQUIRE(present_document.has_value());

    REQUIRE(present_document.value().identities("stations/station").empty());
    REQUIRE_FALSE(present_document.value().key("stations/station", "alpha", "panel/width").has_value());
    REQUIRE(present_document.value().key("stations/station", "alpha", "panel/width").error().code == error_code::unlocatable_key);
}

TEST_CASE("a collection with one instance answers that instance by its identity", "[config]")
{
    const expected<document, error> present_document = folded("one.xml", "<station name=\"alpha\"><panel width=\"10\" scale=\"1.5\"/></station>");
    INFO(why(present_document));
    REQUIRE(present_document.has_value());

    const std::vector<std::string> present = present_document.value().identities("stations/station");
    REQUIRE(present == std::vector<std::string>{"alpha"});

    const expected<std::string, error> scoped = present_document.value().key("stations/station", "alpha", "panel/width");
    REQUIRE(scoped.has_value());
    REQUIRE(scoped.value() == "stations/station[0]/panel/width");

    const expected<std::int64_t, error> width = present_document.value().integer(scoped.value());
    REQUIRE(width.has_value());
    REQUIRE(width.value() == 10);
}

TEST_CASE("a collection with two instances keeps one leaf's value distinct under each", "[config]")
{
    const expected<document, error> present_document = folded("two.xml",
                                                              "<station name=\"alpha\"><panel width=\"10\" scale=\"1.5\"/></station>"
                                                              "<station name=\"beta\"><panel width=\"90\" scale=\"2.5\"/></station>");
    INFO(why(present_document));
    REQUIRE(present_document.has_value());

    REQUIRE(present_document.value().identities("stations/station") == std::vector<std::string>{"alpha", "beta"});

    const expected<std::string, error> first  = present_document.value().key("stations/station", "alpha", "panel/width");
    const expected<std::string, error> second = present_document.value().key("stations/station", "beta", "panel/width");
    REQUIRE(first.has_value());
    REQUIRE(second.has_value());
    REQUIRE(first.value() != second.value());

    const expected<std::int64_t, error> narrow = present_document.value().integer(first.value());
    const expected<std::int64_t, error> wide   = present_document.value().integer(second.value());
    REQUIRE(narrow.has_value());
    REQUIRE(wide.has_value());
    REQUIRE(narrow.value() == 10);
    REQUIRE(wide.value() == 90);
}

TEST_CASE("a read naming no instance is refused rather than answered", "[config]")
{
    const expected<document, error> present_document = folded("unindexed.xml",
                                                              "<station name=\"alpha\"><panel width=\"10\" scale=\"1.5\"/></station>"
                                                              "<station name=\"beta\"><panel width=\"90\" scale=\"2.5\"/></station>");
    INFO(why(present_document));
    REQUIRE(present_document.has_value());

    const expected<std::int64_t, error> crossing = present_document.value().integer("stations/station/panel/width");
    REQUIRE_FALSE(crossing.has_value());
    REQUIRE(crossing.error().code == error_code::instance_required);
    REQUIRE(crossing.error().message.find("stations/station") != std::string::npos);
}

TEST_CASE("a key no declaration names and a kind a leaf was not declared as are told apart", "[config]")
{
    const expected<document, error> present_document = folded("kinds.xml", "<station name=\"alpha\"><panel width=\"10\" scale=\"1.5\"/></station>");
    INFO(why(present_document));
    REQUIRE(present_document.has_value());

    const expected<std::string, error> undeclared = present_document.value().text("window/subtitle");
    REQUIRE_FALSE(undeclared.has_value());
    REQUIRE(undeclared.error().code == error_code::absent_key);

    const expected<bool, error> wrong = present_document.value().flag("stations/station[0]/panel/scale");
    REQUIRE_FALSE(wrong.has_value());
    REQUIRE(wrong.error().code == error_code::mismatched_kind);
}

TEST_CASE("a real leaf written with enough digits to reconstruct it reads back bit for bit", "[config]")
{
    const double authored                            = 0.1 + 0.2;
    const expected<document, error> present_document = folded("real.xml", "<station name=\"alpha\"><panel width=\"1\" scale=\"" + exactly(authored) + "\"/></station>");
    INFO(why(present_document));
    REQUIRE(present_document.has_value());

    const expected<double, error> read = present_document.value().real("stations/station[0]/panel/scale");
    REQUIRE(read.has_value());
    REQUIRE(read.value() == authored);
}

TEST_CASE("an identity two instances share is refused by name", "[config]")
{
    const expected<document, error> present_document = folded("duplicated.xml",
                                                              "<station name=\"alpha\"><panel width=\"10\"/></station>"
                                                              "<station name=\"alpha\"><panel width=\"90\"/></station>");

    REQUIRE_FALSE(present_document.has_value());
    REQUIRE(present_document.error().code == error_code::malformed_source);
    REQUIRE(present_document.error().message.find("stations/station") != std::string::npos);
    REQUIRE(present_document.error().message.find("name") != std::string::npos);
}

TEST_CASE("a collection hanging directly under the root is refused by name", "[config]")
{
    declaration shape("probe");
    shape.collection("station", "name");

    const location at                                = resolve(std::filesystem::path("unused.xml"), std::filesystem::temp_directory_path() / "praxis-config-scoped");
    const expected<document, error> present_document = load(shape, at);

    REQUIRE_FALSE(present_document.has_value());
    REQUIRE(present_document.error().code == error_code::malformed_source);
    REQUIRE(present_document.error().message.find("station") != std::string::npos);
}

TEST_CASE("an instance carrying no identity is refused by name", "[config]")
{
    const expected<document, error> present_document = folded("absent-identity.xml",
                                                              "<station name=\"alpha\"><panel width=\"10\"/></station>"
                                                              "<station><panel width=\"90\"/></station>");

    REQUIRE_FALSE(present_document.has_value());
    REQUIRE(present_document.error().code == error_code::rejected_content);
    REQUIRE(present_document.error().message.find("stations/station") != std::string::npos);
    REQUIRE(present_document.error().message.find("name") != std::string::npos);
}
