#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>
#include <fstream>
#include <iterator>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::config;

namespace {

declaration described()
{
    declaration shape("probe");
    shape.collection("station", "name").field("station/width", field_kind::integer, "0").group("panel").field("panel/scale", field_kind::real, "1.5");
    return shape;
}

std::filesystem::path scratch()
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-config-root-collection";
    std::filesystem::create_directories(directory);
    return directory;
}

std::filesystem::path authored(const std::string &name, std::string_view body)
{
    const std::filesystem::path where = scratch() / name;
    std::ofstream(where, std::ios::binary | std::ios::trunc) << body;
    return where;
}

std::string text_of(const std::filesystem::path &where)
{
    std::ifstream in(where, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

template<typename Outcome>
std::string why(const Outcome &outcome)
{
    return outcome.has_value() ? std::string() : outcome.error().message;
}

bool names(const error &refusal, std::string_view first, std::string_view second)
{
    return refusal.message.find(first) != std::string::npos && refusal.message.find(second) != std::string::npos;
}

constexpr std::string_view alpha_and_beta = "<probe>\n"
                                            "    <station name=\"alpha\" width=\"1\"/>\n"
                                            "    <station name=\"beta\" width=\"2\"/>\n"
                                            "</probe>\n";

}

TEST_CASE("two instances under the root sharing an identity are refused by name", "[config]")
{
    const std::filesystem::path where      = authored("shared.xml", "<probe><station name=\"a\"/><station name=\"a\"/></probe>");
    const expected<document, error> loaded = load(described(), resolve(where, scratch()));

    REQUIRE_FALSE(loaded.has_value());
    REQUIRE(loaded.error().code == error_code::malformed_source);
    REQUIRE(names(loaded.error(), "station", "name"));
}

TEST_CASE("an instance under the root carrying no identity is refused by name", "[config]")
{
    const std::filesystem::path where      = authored("unnamed.xml", "<probe><station name=\"a\" width=\"1\"/><station width=\"2\"/></probe>");
    const expected<document, error> loaded = load(described(), resolve(where, scratch()));

    REQUIRE_FALSE(loaded.has_value());
    REQUIRE(loaded.error().code == error_code::rejected_content);
    REQUIRE(names(loaded.error(), "station", "name"));
}

TEST_CASE("an instance a save names the identity of is created under the root and written into", "[config]")
{
    const location at = resolve(authored("created.xml", "<probe><station name=\"alpha\" width=\"1\"/></probe>"), scratch());

    const std::vector<edit> naming{edit{"station[1]/name", "gamma"}, edit{"station[1]/width", "3"}};
    const expected<void, error> saved = save(described(), at, naming);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const expected<document, error> reloaded = load(described(), at);
    INFO(why(reloaded));
    REQUIRE(reloaded.has_value());
    REQUIRE(reloaded.value().identities("station") == std::vector<std::string>{"alpha", "gamma"});
    const expected<std::string, error> keyed = reloaded.value().key("station", "gamma", "width");
    REQUIRE(keyed.has_value());
    REQUIRE(reloaded.value().integer(keyed.value()).value() == 3);
}

TEST_CASE("an instance a save takes out of the root goes whole and leaves every other byte", "[config]")
{
    const std::filesystem::path where = authored("taken-out.xml", alpha_and_beta);

    const std::vector<edit> one{edit{"station", "alpha", edit_kind::taken_out}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), one);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    std::string only_beta(alpha_and_beta);
    const std::string gone = "\n    <station name=\"alpha\" width=\"1\"/>";
    only_beta.erase(only_beta.find(gone), gone.size());
    REQUIRE(text_of(where) == only_beta);

    const expected<document, error> reloaded = load(described(), resolve(where, scratch()));
    REQUIRE(reloaded.has_value());
    REQUIRE(reloaded.value().identities("station") == std::vector<std::string>{"beta"});
}

TEST_CASE("a value written after a removal from the root lands in the instance it named", "[config]")
{
    const std::filesystem::path where = authored("taken-out-beside.xml", alpha_and_beta);

    const std::vector<edit> changes{edit{"station", "alpha", edit_kind::taken_out}, edit{"station[1]/width", "7"}};
    const expected<void, error> saved = save(described(), resolve(where, scratch()), changes);
    INFO(why(saved));
    REQUIRE(saved.has_value());

    const expected<document, error> reloaded = load(described(), resolve(where, scratch()));
    REQUIRE(reloaded.has_value());
    REQUIRE(reloaded.value().identities("station") == std::vector<std::string>{"beta"});
    REQUIRE(reloaded.value().integer("station[0]/width").value() == 7);
}

TEST_CASE("a starter document leaves a collection under the root out and reads back at the fallbacks", "[config]")
{
    const std::filesystem::path target = scratch() / "starter.xml";
    std::filesystem::remove(target);
    const expected<void, error> written = write_template(described(), target);
    INFO(why(written));
    REQUIRE(written.has_value());
    REQUIRE(text_of(target).find("<panel") != std::string::npos);
    REQUIRE(text_of(target).find("<station") == std::string::npos);

    const expected<document, error> reloaded = load(described(), resolve(target, scratch()));
    INFO(why(reloaded));
    REQUIRE(reloaded.has_value());
    REQUIRE(reloaded.value().real("panel/scale").value() == 1.5);
    REQUIRE(reloaded.value().origin_of("panel/scale").kind == origin_kind::source);
    REQUIRE(reloaded.value().identities("station").empty());
}
