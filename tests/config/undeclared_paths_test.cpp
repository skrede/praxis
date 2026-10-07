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
#include <optional>
#include <iterator>
#include <algorithm>
#include <filesystem>
#include <string_view>
#include <type_traits>
#include <initializer_list>

using namespace praxis;
using namespace praxis::config;

namespace {

declaration described()
{
    declaration shape("probe");
    shape.group("window")
            .field("window/title", field_kind::text, "untitled")
            .field("window/width", field_kind::integer, "800")
            .group("stations")
            .collection("stations/station", "name")
            .field("stations/station/width", field_kind::integer, "0");
    return shape;
}

constexpr std::string_view misspelled = "<probe><window titel=\"b\" width=\"3\"/><extra/></probe>\n";

std::filesystem::path written(const std::string &name, std::string_view body)
{
    const std::filesystem::path directory = std::filesystem::weakly_canonical(std::filesystem::temp_directory_path()) / "praxis-config-undeclared";
    std::filesystem::create_directories(directory);
    const std::filesystem::path where = directory / name;
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

// Whether one line of `log` names every one of `parts`.
bool one_line_names(std::string_view log, std::initializer_list<std::string_view> parts)
{
    for(std::size_t from = 0; from <= log.size();)
    {
        const std::string_view line = log.substr(from, log.find('\n', from) - from);
        if(std::ranges::all_of(parts, [&](std::string_view part) { return line.find(part) != std::string_view::npos; }))
            return true;
        from += line.size() + 1;
    }
    return false;
}

template<typename Read>
std::string answer_of(const Read &read)
{
    if(!read)
        return "error " + std::string(error_name(read.error().code));
    if constexpr(std::is_same_v<std::decay_t<decltype(read.value())>, std::string>)
        return "value " + read.value();
    else
        return "value " + std::to_string(read.value());
}

// What `values` answers for every declared key, the file a value came from aside, and the identities
// of the collection with the key each addresses.
std::vector<std::string> answers(const document &values)
{
    constexpr std::array keys{"window/title", "window/width", "stations/station[0]/name", "stations/station[0]/width", "stations/station[1]/width"};
    std::vector<std::string> answered;
    for(const std::string_view key : keys)
    {
        const value_origin origin = values.origin_of(key);
        answered.push_back(std::string(key) + " " + answer_of(values.text(key)) + " " + answer_of(values.integer(key)) + " origin " + std::to_string(static_cast<int>(origin.kind)) +
                           (origin.layer.empty() ? " unlayered" : " layered") + " held " + std::to_string(values.holds(key)));
    }
    for(const std::string &identity : values.identities("stations/station"))
    {
        const expected<std::string, error> addressed = values.key("stations/station", identity, "width");
        answered.push_back("identity " + identity + " at " + (addressed ? addressed.value() : std::string(error_name(addressed.error().code))));
    }
    return answered;
}

// A document carrying undeclared paths, and the same document with them deleted.
struct same_but_undeclared
{
    const char *name;
    std::string_view carrying;
    std::string_view without;
};

constexpr std::array same_but_undeclared_paths{
        same_but_undeclared{"attributes", "<probe><window titel=\"b\" width=\"3\" colour=\"red\"/></probe>", "<probe><window width=\"3\"/></probe>"},
        same_but_undeclared{"element", "<probe><window width=\"3\"><extra x=\"1\"/></window><extra/></probe>", "<probe><window width=\"3\"></window></probe>"},
        same_but_undeclared{"instance", "<probe><stations><station colour=\"red\"/><station name=\"b\" width=\"4\"/></stations></probe>",
                            "<probe><stations><station/><station name=\"b\" width=\"4\"/></stations></probe>"},
        same_but_undeclared{"only", "<probe><extra/></probe>", "<probe/>"},
        same_but_undeclared{"beside-text", "<probe><window><title lang=\"en\">abc</title></window></probe>", "<probe><window><title>abc</title></window></probe>"},
        same_but_undeclared{"beside-nothing", "<probe><window><title colour=\"red\"/></window></probe>", "<probe><window><title/></window></probe>"},
        same_but_undeclared{"mixed", "<probe><junk>note<x/></junk></probe>", "<probe/>"},
        same_but_undeclared{"text-and-attribute", "<probe><junk a=\"1\">note</junk></probe>", "<probe/>"},
        same_but_undeclared{"repeated-attribute", "<probe><junk a=\"1\" a=\"2\"/></probe>", "<probe/>"},
};

// `levels` elements nested one in the next under the root, at paths nothing declares.
std::string nested_undeclared(std::size_t levels)
{
    std::string body = "<probe>";
    for(std::size_t level = 0; level < levels; ++level)
        body += "<j>";
    for(std::size_t level = 0; level < levels; ++level)
        body += "</j>";
    return body + "</probe>";
}

// The path of the `level`-th of those elements.
std::string nested_path(std::size_t level)
{
    std::string path = "j";
    for(std::size_t below = 1; below < level; ++below)
        path += "/j";
    return path;
}

}

TEST_CASE("a plain load leaves an undeclared path out, reads every declared value, and says nothing", "[config]")
{
    const std::filesystem::path where = written("plain.xml", misspelled);
    std::optional<expected<document, error>> read;
    const std::string said = tests::reported_by([&] { read.emplace(load(described(), at(where))); });

    REQUIRE(read.has_value());
    REQUIRE(read->has_value());
    const document &values = read->value();
    CHECK(values.integer("window/width").value() == 3);
    CHECK(values.origin_of("window/width").kind == origin_kind::source);
    CHECK(values.text("window/title").value() == "untitled");
    CHECK(values.origin_of("window/title").kind == origin_kind::fallback);
    CHECK_FALSE(values.holds("window/titel"));
    CHECK(values.text("window/titel").error().code == error_code::absent_key);
    CHECK(values.origin_of("window/titel").kind == origin_kind::undeclared);
    CHECK_FALSE(values.holds("extra"));
    CHECK(said.empty());
}

TEST_CASE("the answering load names each undeclared path with the declared path nearest to it and reports no failure", "[config]")
{
    const std::filesystem::path where = written("answering.xml", "<probe><window titel=\"b\" width=\"3\"/><extra/><stations><station name=\"a\" nam=\"c\"/></stations></probe>\n");

    for(const expectation carries : {expectation::complete, expectation::partial})
    {
        std::optional<outcome> answered;
        const std::string said = tests::reported_by([&] { answered = load_or_defaults(described(), at(where), carries); });

        REQUIRE(answered.has_value());
        CHECK_FALSE(answered->failure.has_value());
        CHECK(answered->values.integer("window/width").value() == 3);
        CHECK(one_line_names(said, {"warn", "'window/titel'", where.string(), "'window/title'"}));
        CHECK(one_line_names(said, {"warn", "'extra'", where.string()}));
        CHECK(one_line_names(said, {"warn", "'stations/station[0]/nam'", "'stations/station/name'"}));
    }
}

TEST_CASE("a document carrying undeclared paths answers every declared key as the same document without them", "[config]")
{
    for(const same_but_undeclared &one : same_but_undeclared_paths)
    {
        INFO(one.name);
        const expected<document, error> carrying = load(described(), at(written(std::string(one.name) + "-carrying.xml", one.carrying)));
        const expected<document, error> without  = load(described(), at(written(std::string(one.name) + "-without.xml", one.without)));
        INFO((carrying ? std::string() : carrying.error().message));
        REQUIRE(carrying.has_value());
        REQUIRE(without.has_value());
        CHECK(answers(carrying.value()) == answers(without.value()));
        if(std::string_view(one.name) == "instance")
            REQUIRE(carrying.value().key("stations/station", "b", "width").value() == "stations/station[1]/width");
    }
}

TEST_CASE("undeclared elements nested past the depth bound answer as the document without them, the deepest named being the 65th", "[config]")
{
    const std::filesystem::path where        = written("deep-carrying.xml", nested_undeclared(70));
    const expected<document, error> carrying = load(described(), at(where));
    const expected<document, error> without  = load(described(), at(written("deep-without.xml", "<probe/>")));
    std::optional<outcome> answered;
    const std::string said = tests::reported_by([&] { answered = load_or_defaults(described(), at(where), expectation::partial); });

    INFO((carrying ? std::string() : carrying.error().message));
    REQUIRE(carrying.has_value());
    REQUIRE(without.has_value());
    CHECK(answers(carrying.value()) == answers(without.value()));
    CHECK(one_line_names(said, {"warn", "'" + nested_path(65) + "'"}));
    CHECK(said.find(nested_path(66)) == std::string::npos);
}

TEST_CASE("a save into a document carrying an undeclared path writes its edit and leaves that path as it was", "[config]")
{
    const std::filesystem::path where = written("saved.xml", misspelled);

    const std::vector<edit> changes{edit{"window/width", "5"}};
    const expected<void, error> saved = save(described(), at(where), changes);

    INFO((saved ? std::string() : saved.error().message));
    REQUIRE(saved.has_value());
    CHECK(text_of(where) == "<probe><window titel=\"b\" width=\"5\"/><extra/></probe>\n");
    CHECK(load(described(), at(where)).value().integer("window/width").value() == 5);
}
