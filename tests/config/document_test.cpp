#include "held_home.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <fstream>
#include <optional>
#include <algorithm>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::config;

namespace {

std::filesystem::path scratch()
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-config-document";
    std::filesystem::create_directories(directory);
    // The library answers resolved paths, and a temporary directory is not always its own resolved
    // form: macOS reaches it through a symlink and Windows through an abbreviated component. A
    // fixture comparing against the raw path would fail on those two and pass here for no reason.
    return std::filesystem::weakly_canonical(directory);
}

std::filesystem::path written(const std::string &name, std::string_view text)
{
    const std::filesystem::path where = scratch() / name;
    std::ofstream out(where, std::ios::trunc);
    out << text;
    return where;
}

std::filesystem::path home_directory()
{
    const std::filesystem::path directory = scratch() / "home";
    std::filesystem::create_directories(directory);
    return directory;
}

bool carries_a_tilde_component(const std::filesystem::path &of)
{
    return std::find(of.begin(), of.end(), std::filesystem::path("~")) != of.end();
}

declaration one_window()
{
    declaration shape("probe");
    shape.group("window").field("window/title", field_kind::text, "untitled").field("window/width", field_kind::integer, "800");
    return shape;
}

}

TEST_CASE("a declared value is read back out of the file it was written in", "[config]")
{
    const std::filesystem::path where = written("one.xml", "<probe>\n    <window title=\"Overview\" width=\"1280\"/>\n</probe>\n");
    const location at                 = resolve(where, scratch());

    const expected<document, error> loaded = load(one_window(), at);
    REQUIRE(loaded.has_value());

    const expected<std::string, error> title = loaded.value().text("window/title");
    REQUIRE(title.has_value());
    REQUIRE(title.value() == "Overview");

    REQUIRE(loaded.value().source() == std::filesystem::weakly_canonical(where));
    REQUIRE(loaded.value().source().is_absolute());
}

TEST_CASE("a leaf the document omits carries the fallback the declaration named", "[config]")
{
    const std::filesystem::path where      = written("omitted.xml", "<probe>\n    <window width=\"1280\"/>\n</probe>\n");
    const expected<document, error> loaded = load(one_window(), resolve(where, scratch()));
    REQUIRE(loaded.has_value());

    const expected<std::string, error> title = loaded.value().text("window/title");
    REQUIRE(title.has_value());
    REQUIRE(title.value() == "untitled");
    REQUIRE(loaded.value().origin_of("window/title").kind == origin_kind::fallback);

    const expected<std::int64_t, error> width = loaded.value().integer("window/width");
    REQUIRE(width.has_value());
    REQUIRE(width.value() == 1280);
    REQUIRE(loaded.value().origin_of("window/width").kind == origin_kind::source);
    REQUIRE_FALSE(loaded.value().origin_of("window/width").layer.empty());
}

TEST_CASE("an absolute path resolves to its weakly canonical form", "[config]")
{
    const std::filesystem::path where = scratch() / "sub" / ".." / "absolute.xml";
    const location at                 = resolve(where, scratch());

    REQUIRE(at.given == where);
    REQUIRE(at.resolved == std::filesystem::weakly_canonical(where));
    REQUIRE(at.resolved == scratch() / "absolute.xml");
}

TEST_CASE("a path that is not absolute resolves against the base and not the working directory", "[config]")
{
    const std::filesystem::path base = scratch();
    const location at                = resolve(std::filesystem::path("relative.xml"), base);

    REQUIRE(at.resolved == std::filesystem::weakly_canonical(base / "relative.xml"));
    REQUIRE(at.resolved.parent_path() == std::filesystem::weakly_canonical(base));
    REQUIRE(at.resolved != std::filesystem::weakly_canonical(std::filesystem::current_path() / "relative.xml"));
}

TEST_CASE("a leading tilde names the home directory and what follows it hangs below", "[config]")
{
    const held_home during{home_directory().string()};
    const location at = resolve(std::filesystem::path("~/inner/tilde.xml"), scratch());

    REQUIRE(at.given == std::filesystem::path("~/inner/tilde.xml"));
    REQUIRE(at.resolved == std::filesystem::weakly_canonical(home_directory() / "inner" / "tilde.xml"));
    REQUIRE_FALSE(carries_a_tilde_component(at.resolved));
}

TEST_CASE("with every home variable carrying nothing, a leading tilde stays as it was written", "[config]")
{
    const held_home during{std::string()};
    const location at = resolve(std::filesystem::path("~/inner/tilde.xml"), scratch());

    REQUIRE(at.given == std::filesystem::path("~/inner/tilde.xml"));
    REQUIRE(at.resolved == std::filesystem::weakly_canonical(scratch() / "~" / "inner" / "tilde.xml"));
    REQUIRE(carries_a_tilde_component(at.resolved));
}

TEST_CASE("with no home variable there at all, a leading tilde stays as it was written", "[config]")
{
    const held_home during{std::nullopt};
    const location at = resolve(std::filesystem::path("~/inner/tilde.xml"), scratch());

    REQUIRE(at.given == std::filesystem::path("~/inner/tilde.xml"));
    REQUIRE(at.resolved == std::filesystem::weakly_canonical(scratch() / "~" / "inner" / "tilde.xml"));
    REQUIRE(carries_a_tilde_component(at.resolved));
}

TEST_CASE("a tilde followed by a name is an ordinary directory component, and a bare tilde resolves", "[config]")
{
    const held_home during{home_directory().string()};
    const location at = resolve(std::filesystem::path("~someone/x.xml"), scratch());

    REQUIRE(at.resolved == std::filesystem::weakly_canonical(scratch() / "~someone" / "x.xml"));
    REQUIRE(resolve(std::filesystem::path(), scratch()).resolved.is_absolute());
    REQUIRE(resolve(std::filesystem::path("~"), scratch()).resolved == std::filesystem::weakly_canonical(home_directory()));
}

TEST_CASE("a load from a path that is not there names the resolved path", "[config]")
{
    const location at                      = resolve(std::filesystem::path("absent.xml"), scratch());
    const expected<document, error> loaded = load(one_window(), at);

    REQUIRE_FALSE(loaded.has_value());
    REQUIRE(loaded.error().code == error_code::absent_source);
    REQUIRE(loaded.error().message.find(at.resolved.string()) != std::string::npos);
}

TEST_CASE("a document that does not parse is refused as malformed", "[config]")
{
    const std::filesystem::path where      = written("broken.xml", "<probe><window title=\"Overview\"\n");
    const expected<document, error> loaded = load(one_window(), resolve(where, scratch()));

    REQUIRE_FALSE(loaded.has_value());
    REQUIRE(loaded.error().code == error_code::malformed_source);
    REQUIRE_FALSE(loaded.error().message.empty());
}

TEST_CASE("every error code carries a name of its own", "[config]")
{
    REQUIRE(std::string_view(error_name(error_code::unreadable_source)) == "unreadable_source");
    REQUIRE(std::string_view(error_name(error_code::instance_required)) == "instance_required");
    REQUIRE(std::string_view(error_name(error_code::unwritable_target)) == "unwritable_target");
}
