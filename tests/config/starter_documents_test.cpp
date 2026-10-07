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
#include <initializer_list>

using namespace praxis;
using namespace praxis::config;

namespace {

std::filesystem::path fresh(const std::string &name)
{
    const std::filesystem::path directory = std::filesystem::weakly_canonical(std::filesystem::temp_directory_path()) / "praxis-config-starter";
    std::filesystem::create_directories(directory);
    std::filesystem::remove(directory / name);
    return directory / name;
}

std::string contents(const std::filesystem::path &where)
{
    std::ifstream in(where, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

bool names_all(const std::string &message, std::initializer_list<const char *> paths)
{
    for(const char *path : paths)
        if(message.find(std::string("'") + path + "'") == std::string::npos)
            return false;
    return true;
}

}

TEST_CASE("a declaration that cannot describe a keyspace is refused by name wherever it is used", "[config]")
{
    declaration shape("probe");
    shape.field("loose/leaf", field_kind::text, "x")
            .group("window")
            .field("window/title", field_kind::text, "a")
            .field("window/title", field_kind::text, "b")
            .field("window/width", field_kind::integer, "1")
            .field("window/width/unit", field_kind::text, "m")
            .collection("window/station", "name")
            .field("window/station/name", field_kind::text, "");
    const std::initializer_list<const char *> mistaken = {"loose/leaf", "window/title", "window/width/unit", "window/station/name"};

    const std::filesystem::path where = fresh("refused-declaration.xml");
    std::ofstream(where) << "<probe/>";
    const expected<document, error> loaded = load(shape, resolve(where, where.parent_path()));
    REQUIRE_FALSE(loaded.has_value());
    REQUIRE((loaded.error().code == error_code::malformed_source && names_all(loaded.error().message, mistaken)));

    const std::filesystem::path target  = fresh("refused-declaration-starter.xml");
    const expected<void, error> written = write_template(shape, target);
    REQUIRE_FALSE(written.has_value());
    REQUIRE((written.error().code == error_code::malformed_source && names_all(written.error().message, mistaken)));
    REQUIRE_FALSE(std::filesystem::exists(target));
}

TEST_CASE("a starter document is not written where a name or a fallback cannot be carried as XML", "[config]")
{
    const std::filesystem::path target = fresh("unwritable-starter.xml");

    const expected<void, error> spaced = write_template(declaration("my probe").field("title", field_kind::text, "x"), target);
    REQUIRE_FALSE(spaced.has_value());
    REQUIRE(spaced.error().code == error_code::malformed_source);
    REQUIRE(names_all(spaced.error().message, {"my probe"}));

    const expected<void, error> segment = write_template(declaration("probe").group("window").field("window/full title", field_kind::text, "x"), target);
    REQUIRE_FALSE(segment.has_value());
    REQUIRE(segment.error().code == error_code::malformed_source);
    REQUIRE(names_all(segment.error().message, {"full title"}));

    const expected<void, error> control = write_template(declaration("probe").group("window").field("window/title", field_kind::text, "a\rb"), target);
    REQUIRE_FALSE(control.has_value());
    REQUIRE(control.error().code == error_code::malformed_source);
    REQUIRE(names_all(control.error().message, {"window/title"}));
    REQUIRE_FALSE(std::filesystem::exists(target));
}

TEST_CASE("a starter document reads back every fallback it was written from, blank and escaped ones included", "[config]")
{
    declaration shape("probe");
    shape.group("window").field("window/empty", field_kind::text, "").field("window/blank", field_kind::text, " \t\n ").field("window/marked", field_kind::text, "<a> & \"b\"");

    const std::filesystem::path target = fresh("blank-starter.xml");
    REQUIRE(write_template(shape, target).has_value());

    const expected<document, error> loaded = load(shape, resolve(target, target.parent_path()));
    REQUIRE(loaded.has_value());
    for(const char *key : {"window/empty", "window/blank", "window/marked"})
        REQUIRE(loaded.value().origin_of(key).kind == origin_kind::source);
    REQUIRE(loaded.value().text("window/empty").value().empty());
    REQUIRE(loaded.value().text("window/blank").value() == " \t\n ");
    REQUIRE(loaded.value().text("window/marked").value() == "<a> & \"b\"");
}

TEST_CASE("a starter document writes its groups in path order, one segment at a time", "[config]")
{
    declaration shape("probe");
    shape.group("a-b").field("a-b/x", field_kind::text, "1").group("a").field("a/y", field_kind::text, "2");

    const std::filesystem::path target = fresh("ordered-starter.xml");
    REQUIRE(write_template(shape, target).has_value());

    const std::string text = contents(target);
    REQUIRE(text.find("<a>") != std::string::npos);
    REQUIRE(text.find("<a>") < text.find("<a-b>"));
}

TEST_CASE("a declaration whose space is empty is refused by name by a load and a starter document, and a save into its document writes nothing", "[config]")
{
    declaration shape("");
    shape.group("a").field("a/b", field_kind::text, "FB");

    const std::filesystem::path where = fresh("empty-space.xml");
    std::ofstream(where) << "<a><b>v</b></a>";
    const expected<document, error> loaded = load(shape, resolve(where, where.parent_path()));
    REQUIRE_FALSE(loaded.has_value());
    CHECK(loaded.error().code == error_code::malformed_source);
    CHECK(loaded.error().message.find("the space is empty") != std::string::npos);

    const std::filesystem::path target  = fresh("empty-space-starter.xml");
    const expected<void, error> written = write_template(shape, target);
    REQUIRE_FALSE(written.has_value());
    CHECK(written.error().code == error_code::malformed_source);
    CHECK(written.error().message.find("the space is empty") != std::string::npos);
    CHECK_FALSE(std::filesystem::exists(target));

    const expected<void, error> saved = save(shape, resolve(where, where.parent_path()), std::vector<edit>{edit{"a/b", "Z"}});
    REQUIRE_FALSE(saved.has_value());
    CHECK(contents(where) == "<a><b>v</b></a>");
}
