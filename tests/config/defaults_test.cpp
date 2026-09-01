#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/scene/log_buffer.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <memory>
#include <vector>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <filesystem>
#include <string_view>
#include <system_error>

using namespace praxis;
using namespace praxis::config;

namespace {

std::filesystem::path scratch()
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-config-defaults";
    std::filesystem::create_directories(directory);
    // The library answers resolved paths, and a temporary directory is not always its own resolved
    // form: macOS reaches it through a symlink and Windows through an abbreviated component. A
    // fixture comparing against the raw path would fail on those two and pass here for no reason.
    return std::filesystem::weakly_canonical(directory);
}

std::filesystem::path written(const std::string &name, std::string_view text)
{
    const std::filesystem::path where = scratch() / name;
    std::ofstream out(where, std::ios::trunc | std::ios::binary);
    out << text;
    return where;
}

std::string contents(const std::filesystem::path &where)
{
    std::ifstream in(where, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

bool writable(const std::filesystem::path &directory)
{
    const std::filesystem::path probe = directory / "writable.probe";
    std::ofstream out(probe, std::ios::trunc);
    const bool opened = out.is_open();
    out.close();

    std::error_code ignored;
    std::filesystem::remove(probe, ignored);
    return opened;
}

declaration two_fields()
{
    declaration shape("probe");
    shape.group("window").field("window/title", field_kind::text, "untitled").field("window/width", field_kind::integer, "800");
    return shape;
}

declaration three_fields()
{
    declaration shape("probe");
    shape.group("window").field("window/title", field_kind::text, "untitled").field("window/width", field_kind::integer, "800").choice("window/mode", {"fast", "careful"}, "fast");
    return shape;
}

outcome answered_for(const declaration &shape, const std::string &name, std::string_view text)
{
    return load_or_defaults(shape, resolve(written(name, text), scratch()));
}

std::shared_ptr<scene::log_buffer> ring_on_the_current_logger()
{
    auto messages = std::make_shared<scene::log_buffer>(scene::default_log_capacity);
    scene::install_log_sink(messages);

    return messages;
}

bool carries(const std::vector<scene::log_entry> &drained, scene::severity level, std::string_view fragment)
{
    for(const scene::log_entry &entry : drained)
        if(entry.level == level && entry.text.find(fragment) != std::string::npos)
            return true;
    return false;
}

}

TEST_CASE("a configuration file that is not there still answers, and says so where an operator sees it", "[config]")
{
    const location at                                 = resolve(std::filesystem::path("absent.xml"), scratch());
    const std::shared_ptr<scene::log_buffer> messages = ring_on_the_current_logger();

    const outcome answered = load_or_defaults(two_fields(), at);

    REQUIRE(answered.failure.has_value());
    REQUIRE(answered.failure->code == error_code::absent_source);

    const expected<std::string, error> title  = answered.values.text("window/title");
    const expected<std::int64_t, error> width = answered.values.integer("window/width");
    REQUIRE(title.has_value());
    REQUIRE(title.value() == "untitled");
    REQUIRE(width.has_value());
    REQUIRE(width.value() == 800);

    REQUIRE(carries(messages->drain(), scene::severity::warning, at.resolved.string()));
}

TEST_CASE("a path naming a directory answers the fallbacks and is refused as unreadable", "[config]")
{
    const std::filesystem::path where = scratch() / "as-a-directory.xml";
    std::filesystem::create_directories(where);

    const std::shared_ptr<scene::log_buffer> messages = ring_on_the_current_logger();
    outcome answered                                  = load_or_defaults(two_fields(), resolve(where, scratch()));

    REQUIRE(answered.failure.has_value());
    REQUIRE(answered.failure->code == error_code::unreadable_source);
    REQUIRE(answered.values.text("window/title").value() == "untitled");
    REQUIRE(carries(messages->drain(), scene::severity::error, where.string()));
}

TEST_CASE("a configuration file of zero bytes answers the fallbacks and is refused as empty", "[config]")
{
    const outcome answered = answered_for(two_fields(), "empty.xml", "");

    REQUIRE(answered.failure.has_value());
    REQUIRE(answered.failure->code == error_code::empty_source);
    REQUIRE(answered.values.integer("window/width").value() == 800);
}

TEST_CASE("a configuration file that does not parse answers the fallbacks and is refused as malformed", "[config]")
{
    const outcome answered = answered_for(two_fields(), "unparsable.xml", "<probe><window title=\"Overview\"\n");

    REQUIRE(answered.failure.has_value());
    REQUIRE(answered.failure->code == error_code::malformed_source);
    REQUIRE(answered.values.text("window/title").value() == "untitled");
}

TEST_CASE("a configuration file whose root is not the declared space is refused as a mismatched space", "[config]")
{
    const outcome answered = answered_for(two_fields(), "elsewhere.xml", "<elsewhere>\n    <window title=\"Overview\"/>\n</elsewhere>\n");

    REQUIRE(answered.failure.has_value());
    REQUIRE(answered.failure->code == error_code::mismatched_space);
    REQUIRE(answered.values.text("window/title").value() == "untitled");
}

TEST_CASE("a value outside a declared enumeration is refused as rejected content", "[config]")
{
    const outcome answered = answered_for(three_fields(), "reckless.xml", "<probe>\n    <window mode=\"reckless\"/>\n</probe>\n");

    REQUIRE(answered.failure.has_value());
    REQUIRE(answered.failure->code == error_code::rejected_content);
    REQUIRE(answered.values.text("window/mode").value() == "fast");
}

TEST_CASE("a value of the wrong kind for a declared field is refused as rejected content", "[config]")
{
    const outcome answered = answered_for(three_fields(), "wide.xml", "<probe>\n    <window width=\"wide\"/>\n</probe>\n");

    REQUIRE(answered.failure.has_value());
    REQUIRE(answered.failure->code == error_code::rejected_content);
    REQUIRE(answered.values.integer("window/width").value() == 800);
}

TEST_CASE("a configuration file that omits keys is not a failure, and the count it substituted is announced", "[config]")
{
    const std::shared_ptr<scene::log_buffer> messages = ring_on_the_current_logger();
    const outcome answered                            = answered_for(three_fields(), "partial.xml", "<probe>\n    <window width=\"1280\"/>\n</probe>\n");

    REQUIRE_FALSE(answered.failure.has_value());
    REQUIRE(answered.values.integer("window/width").value() == 1280);
    REQUIRE(answered.values.text("window/title").value() == "untitled");
    REQUIRE(answered.values.text("window/mode").value() == "fast");

    REQUIRE(carries(messages->drain(), scene::severity::info, "2 of 3 declared values"));
}

TEST_CASE("a plain load says nothing at all, and only the answering load reports where it read from", "[config]")
{
    const scene::severity held                        = scene::reporting_level();
    const std::shared_ptr<scene::log_buffer> messages = ring_on_the_current_logger();
    const declaration shape                           = two_fields();
    const location at                                 = resolve(written("silent.xml", "<probe><window title=\"named\" width=\"640\"/></probe>"), scratch());

    scene::set_reporting_level(scene::severity::debug);
    REQUIRE(load(shape, at).has_value());
    const std::vector<scene::log_entry> plainly = messages->drain();

    static_cast<void>(load_or_defaults(shape, at));
    const std::vector<scene::log_entry> answering = messages->drain();
    scene::set_reporting_level(held);

    CHECK(plainly.empty());
    CHECK(carries(answering, scene::severity::info, at.resolved.string()));
}

TEST_CASE("what the configuration engine remarks on arrives at the detail level or not at all", "[config]")
{
    const scene::severity held = scene::reporting_level();
    const declaration shape    = three_fields();
    const location at          = resolve(written("engine.xml", "<probe>\n    <window width=\"1280\"/>\n</probe>\n"), scratch());

    scene::set_reporting_level(scene::severity::info);
    const std::shared_ptr<scene::log_buffer> ordinary = ring_on_the_current_logger();
    static_cast<void>(load_or_defaults(shape, at));
    const std::vector<scene::log_entry> plainly = ordinary->drain();

    scene::set_reporting_level(scene::severity::debug);
    const std::shared_ptr<scene::log_buffer> detailed = ring_on_the_current_logger();
    static_cast<void>(load_or_defaults(shape, at));
    const std::vector<scene::log_entry> in_detail = detailed->drain();
    scene::set_reporting_level(held);

    for(const scene::log_entry &entry : plainly)
        CHECK(entry.level <= scene::severity::info);
    CHECK(carries(plainly, scene::severity::info, "2 of 3 declared values"));
    CHECK(carries(in_detail, scene::severity::debug, "degraded"));
    for(const scene::log_entry &entry : in_detail)
        CHECK(entry.text.find(".staging") == std::string::npos);
}

TEST_CASE("the count of substituted values is published at a level the logger admits without being changed", "[config]")
{
    REQUIRE(scene::reporting_level() <= scene::severity::info);
}

TEST_CASE("a read answers whether its value came from the file, from a fallback, or from no declaration at all", "[config]")
{
    const outcome answered = answered_for(three_fields(), "origins.xml", "<probe>\n    <window width=\"1280\"/>\n</probe>\n");

    REQUIRE_FALSE(answered.failure.has_value());
    REQUIRE(answered.values.origin_of("window/width").kind == origin_kind::source);
    REQUIRE_FALSE(answered.values.origin_of("window/width").layer.empty());
    REQUIRE(answered.values.origin_of("window/title").kind == origin_kind::fallback);
    REQUIRE(answered.values.origin_of("window/title").layer.empty());
    REQUIRE(answered.values.origin_of("window/undeclared").kind == origin_kind::undeclared);
}

TEST_CASE("a key crossing a populated collection without an ordinal is not answered as a fallback", "[config]")
{
    declaration shape("probe");
    shape.group("stations").collection("stations/station", "name").field("stations/station/width", field_kind::integer, "0");

    const outcome answered = load_or_defaults(
            shape, resolve(written("instances.xml", "<probe>\n    <stations>\n        <station name=\"alpha\" width=\"10\"/>\n    </stations>\n</probe>\n"), scratch()));

    REQUIRE_FALSE(answered.failure.has_value());
    REQUIRE(answered.values.origin_of("stations/station[0]/width").kind == origin_kind::source);
    REQUIRE(answered.values.origin_of("stations/station/width").kind == origin_kind::instance_required);
}

TEST_CASE("every failure mode returns rather than throwing", "[config]")
{
    REQUIRE_NOTHROW(load_or_defaults(two_fields(), resolve(std::filesystem::path("never-written.xml"), scratch())));
    REQUIRE_NOTHROW(answered_for(two_fields(), "nothrow-empty.xml", ""));
    REQUIRE_NOTHROW(answered_for(two_fields(), "nothrow-unparsable.xml", "<probe"));
    REQUIRE_NOTHROW(answered_for(two_fields(), "nothrow-elsewhere.xml", "<elsewhere/>"));
    REQUIRE_NOTHROW(answered_for(three_fields(), "nothrow-reckless.xml", "<probe><window mode=\"reckless\"/></probe>"));
}

TEST_CASE("a refused source answers every declared key from its fallback and names the refusal in the same outcome", "[config]")
{
    const outcome answered = answered_for(three_fields(), "refused-and-answered.xml", "<probe><window title=\"Overview\"\n");

    const expected<std::string, error> title  = answered.values.text("window/title");
    const expected<std::int64_t, error> width = answered.values.integer("window/width");
    const expected<std::string, error> mode   = answered.values.text("window/mode");

    REQUIRE(answered.failure.has_value());
    REQUIRE(std::string_view(error_name(answered.failure->code)) == "malformed_source");

    REQUIRE(title.has_value());
    REQUIRE(width.has_value());
    REQUIRE(mode.has_value());
    REQUIRE(title.value() == "untitled");
    REQUIRE(width.value() == 800);
    REQUIRE(mode.value() == "fast");
    REQUIRE(answered.values.origin_of("window/title").kind == origin_kind::fallback);
    REQUIRE(answered.values.origin_of("window/width").kind == origin_kind::fallback);
    REQUIRE(answered.values.origin_of("window/mode").kind == origin_kind::fallback);
}

TEST_CASE("a starter document generated from the declaration reads back with every field at its fallback", "[config]")
{
    const std::filesystem::path where = scratch() / "starter.xml";
    std::filesystem::remove(where);

    REQUIRE(write_template(three_fields(), where).has_value());
    REQUIRE(std::filesystem::is_regular_file(where));

    const outcome answered = load_or_defaults(three_fields(), resolve(where, scratch()));
    REQUIRE_FALSE(answered.failure.has_value());
    REQUIRE(answered.values.text("window/title").value() == "untitled");
    REQUIRE(answered.values.integer("window/width").value() == 800);
    REQUIRE(answered.values.text("window/mode").value() == "fast");
    REQUIRE(answered.values.origin_of("window/title").kind == origin_kind::source);
}

TEST_CASE("generating over a document that is already there refuses and leaves it byte for byte", "[config]")
{
    const std::filesystem::path where = written("hand-written.xml", "<!-- kept -->\n<probe>\n    <window title=\"Overview\"/>\n</probe>\n");
    const std::string before          = contents(where);

    const expected<void, error> refused = write_template(three_fields(), where);

    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error().code == error_code::unwritable_target);
    REQUIRE(refused.error().message.find(where.string()) != std::string::npos);
    REQUIRE(contents(where) == before);
}

TEST_CASE("generating into a directory that cannot be written refuses and leaves nothing behind", "[config]")
{
    const std::filesystem::path sealed = scratch() / "sealed";
    std::filesystem::remove_all(sealed);
    std::filesystem::create_directories(sealed);
    std::filesystem::permissions(sealed, std::filesystem::perms::owner_read | std::filesystem::perms::owner_exec, std::filesystem::perm_options::replace);

    if(writable(sealed))
    {
        std::filesystem::permissions(sealed, std::filesystem::perms::owner_all, std::filesystem::perm_options::replace);
        SKIP("this process writes into a directory carrying no write permission, so the refusal cannot be staged");
    }

    const expected<void, error> refused = write_template(three_fields(), sealed / "starter.xml");

    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error().code == error_code::unwritable_target);

    std::filesystem::permissions(sealed, std::filesystem::perms::owner_all, std::filesystem::perm_options::replace);
    REQUIRE(std::filesystem::is_empty(sealed));
}

TEST_CASE("a file that goes away between the moment it is resolved and the moment it is read answers every fallback", "[config]")
{
    const std::filesystem::path where = written("vanishing.xml", "<probe>\n    <window title=\"Overview\" width=\"1280\"/>\n</probe>\n");
    const location at                 = resolve(where, scratch());
    REQUIRE(std::filesystem::remove(where));

    const outcome answered = load_or_defaults(three_fields(), at);

    REQUIRE(answered.failure.has_value());
    REQUIRE(answered.failure->code == error_code::absent_source);
    REQUIRE(answered.values.text("window/title").value() == "untitled");
    REQUIRE(answered.values.integer("window/width").value() == 800);
    REQUIRE(answered.values.text("window/mode").value() == "fast");
    REQUIRE(answered.values.origin_of("window/width").kind == origin_kind::fallback);
}

TEST_CASE("a file replaced by a directory between the moment it is resolved and the moment it is read answers every fallback", "[config]")
{
    const std::filesystem::path where = written("swapped.xml", "<probe>\n    <window title=\"Overview\" width=\"1280\"/>\n</probe>\n");
    const location at                 = resolve(where, scratch());
    std::filesystem::remove_all(where);
    std::filesystem::create_directories(where);

    const outcome answered = load_or_defaults(three_fields(), at);

    REQUIRE(answered.failure.has_value());
    REQUIRE(answered.failure->code == error_code::unreadable_source);
    REQUIRE(answered.values.text("window/title").value() == "untitled");
    REQUIRE(answered.values.integer("window/width").value() == 800);
}
