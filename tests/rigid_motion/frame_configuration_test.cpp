#include "praxis/rigid_motion/configuration.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <Eigen/Core>

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <optional>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

constexpr std::string_view at = "arrangement";

const std::vector<std::string> &objects()
{
    static const std::vector<std::string> named{"base", "upper", "tip"};
    return named;
}

config::declaration described()
{
    config::declaration shape("probe");
    declare_arrangement(shape, at);
    return shape;
}

std::filesystem::path scratch()
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-frame-configuration";
    std::filesystem::create_directories(directory);
    return directory;
}

// A starter document names no instance of a collection, so an arrangement is authored with its
// instances written out and read from there.
std::filesystem::path authored(const std::string &name, std::string_view body)
{
    const std::filesystem::path where = scratch() / name;
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<probe><arrangement>" << body << "</arrangement></probe>\n";
    return where;
}

std::string placed(std::string_view name, std::string_view parent)
{
    return "<object name=\"" + std::string(name) + "\" parent=\"" + std::string(parent) +
            "\" euler_order=\"ZYX\"><position x=\"0\" y=\"0\" z=\"0\"/><euler x=\"0\" y=\"0\" z=\"0\"/></object>";
}

std::string three(std::string_view first, std::string_view second, std::string_view third)
{
    return placed("base", first) + placed("upper", second) + placed("tip", third);
}

config::document loaded(const std::filesystem::path &where)
{
    const expected<config::document, config::error> read = config::load(described(), config::resolve(where, scratch()));
    INFO((read ? std::string() : read.error().message));
    REQUIRE(read.has_value());
    return read.value();
}

expected<frame_window::settings, config::error> read_back(const std::filesystem::path &where)
{
    return read_arrangement(loaded(where), at, objects());
}

frame_window::placement made(axis_order order, const Eigen::Vector3f &position, const Eigen::Vector3f &euler, std::optional<std::size_t> parent)
{
    frame_window::placement one;
    one.order         = order;
    one.position      = position;
    one.euler_degrees = euler;
    one.parent        = parent;
    return one;
}

void require_same(const frame_window::placement &read, const frame_window::placement &written)
{
    REQUIRE(read.order == written.order);
    REQUIRE(read.position.x() == written.position.x());
    REQUIRE(read.position.y() == written.position.y());
    REQUIRE(read.position.z() == written.position.z());
    REQUIRE(read.euler_degrees.x() == written.euler_degrees.x());
    REQUIRE(read.euler_degrees.y() == written.euler_degrees.y());
    REQUIRE(read.euler_degrees.z() == written.euler_degrees.z());
    REQUIRE(read.parent == written.parent);
}

}

TEST_CASE("every arrangement field written through the declared keys reads back as it was set", "[rigid_motion][configuration]")
{
    const std::filesystem::path where = authored("round-trip.xml", three("", "", ""));

    frame_window::settings written;
    written.objects.push_back(made(axis_order::xyz, {0.5f, 1.5f, -2.5f}, {10.f, -20.f, 30.f}, std::nullopt));
    written.objects.push_back(made(axis_order::yzy, {4.f, 5.f, 6.f}, {1.f, 2.f, 3.f}, std::size_t{0}));
    written.objects.push_back(made(axis_order::zxz, {-7.f, 8.f, 9.5f}, {-45.f, 0.25f, 90.f}, std::size_t{1}));

    const std::vector<config::edit> changes   = write_arrangement(loaded(where), frame_window::settings(), written, at, objects());
    const expected<void, config::error> saved = config::save(described(), config::resolve(where, scratch()), changes);
    INFO((saved ? std::string() : saved.error().message));
    REQUIRE(saved.has_value());

    const expected<frame_window::settings, config::error> read = read_back(where);
    INFO((read ? std::string() : read.error().message));
    REQUIRE(read.has_value());
    REQUIRE(read.value().objects.size() == written.objects.size());
    for(std::size_t which = 0u; which < written.objects.size(); ++which)
    {
        INFO(objects()[which]);
        require_same(read.value().objects[which], written.objects[which]);
    }
}

TEST_CASE("a placement for an object the document carries no instance for is written into a new instance", "[rigid_motion][configuration]")
{
    const std::filesystem::path where = authored("appended.xml", placed("upper", ""));

    frame_window::settings written;
    written.objects.push_back(made(axis_order::xyz, {0.5f, 1.5f, -2.5f}, {10.f, -20.f, 30.f}, std::nullopt));
    written.objects.push_back(made(axis_order::yzy, {4.f, 5.f, 6.f}, {1.f, 2.f, 3.f}, std::size_t{0}));
    written.objects.push_back(made(axis_order::zxz, {-7.f, 8.f, 9.5f}, {-45.f, 0.25f, 90.f}, std::size_t{1}));

    const std::vector<config::edit> changes   = write_arrangement(loaded(where), frame_window::settings(), written, at, objects());
    const expected<void, config::error> saved = config::save(described(), config::resolve(where, scratch()), changes);
    INFO((saved ? std::string() : saved.error().message));
    REQUIRE(saved.has_value());

    const expected<frame_window::settings, config::error> read = read_back(where);
    INFO((read ? std::string() : read.error().message));
    REQUIRE(read.has_value());
    REQUIRE(read.value().objects.size() == written.objects.size());
    for(std::size_t which = 0u; which < written.objects.size(); ++which)
    {
        INFO(objects()[which]);
        require_same(read.value().objects[which], written.objects[which]);
    }

    const std::vector<std::string> named = loaded(where).identities(std::string(at) + "/object");
    REQUIRE(named.size() == objects().size());
    REQUIRE(named[0] == "upper");
    REQUIRE(named[1] == "base");
    REQUIRE(named[2] == "tip");
}

TEST_CASE("a save writes only the leaf a placement moved on, and what it leaves out still comes from the caller", "[rigid_motion][configuration]")
{
    const std::filesystem::path where = authored("one-leaf.xml", placed("upper", ""));

    frame_window::settings opening;
    opening.objects.push_back(made(axis_order::xyz, {0.5f, 1.5f, -2.5f}, {10.f, -20.f, 30.f}, std::nullopt));
    opening.objects.push_back(made(axis_order::yzy, {4.f, 5.f, 6.f}, {1.f, 2.f, 3.f}, std::nullopt));
    opening.objects.push_back(made(axis_order::zxz, {-7.f, 8.f, 9.5f}, {-45.f, 0.25f, 90.f}, std::size_t{0}));

    const expected<frame_window::settings, config::error> opened = read_arrangement(loaded(where), at, objects(), opening);
    INFO((opened ? std::string() : opened.error().message));
    REQUIRE(opened.has_value());

    frame_window::settings moved = opened.value();
    moved.objects[0].position.z() += 1.25f;

    const std::vector<config::edit> changes = write_arrangement(loaded(where), opened.value(), moved, at, objects());
    REQUIRE(changes.size() == 2u);
    REQUIRE(changes.front().key == std::string(at) + "/object[1]/name");
    REQUIRE(changes.front().value == "base");
    REQUIRE(changes.back().key == std::string(at) + "/object[1]/position/z");
    REQUIRE(changes.back().value == "-1.25");

    const expected<void, config::error> saved = config::save(described(), config::resolve(where, scratch()), changes);
    INFO((saved ? std::string() : saved.error().message));
    REQUIRE(saved.has_value());

    const config::document written       = loaded(where);
    const std::vector<std::string> named = written.identities(std::string(at) + "/object");
    REQUIRE(named.size() == 2u);
    REQUIRE(named[0] == "upper");
    REQUIRE(named[1] == "base");

    REQUIRE(written.origin_of(std::string(at) + "/object[1]/position/z").kind == config::origin_kind::source);
    REQUIRE(written.origin_of(std::string(at) + "/object[1]/position/x").kind == config::origin_kind::fallback);
    REQUIRE(written.origin_of(std::string(at) + "/object[1]/euler_order").kind == config::origin_kind::fallback);

    const expected<frame_window::settings, config::error> again = read_arrangement(written, at, objects(), opening);
    INFO((again ? std::string() : again.error().message));
    REQUIRE(again.has_value());
    require_same(again.value().objects[0], moved.objects[0]);

    REQUIRE(write_arrangement(written, again.value(), moved, at, objects()).empty());
}

TEST_CASE("an object the document names no parent for is expressed in the world frame", "[rigid_motion][configuration]")
{
    const std::filesystem::path where = authored("no-parent.xml", "<object name=\"base\"/>" + placed("upper", "base") + "<object name=\"tip\"/>");

    const expected<frame_window::settings, config::error> read = read_back(where);
    INFO((read ? std::string() : read.error().message));
    REQUIRE(read.has_value());
    REQUIRE_FALSE(read.value().objects[0].parent.has_value());
    REQUIRE(read.value().objects[1].parent == std::optional<std::size_t>(std::size_t{0}));
    REQUIRE_FALSE(read.value().objects[2].parent.has_value());
}

TEST_CASE("a parent naming no object of the arrangement is refused by that name", "[rigid_motion][configuration]")
{
    const std::filesystem::path where = authored("unknown-parent.xml", three("", "forearm", ""));

    const expected<frame_window::settings, config::error> read = read_back(where);
    REQUIRE_FALSE(read.has_value());
    REQUIRE(read.error().code == config::error_code::rejected_content);
    REQUIRE(read.error().message.find("forearm") != std::string::npos);
}

TEST_CASE("a chain of parents that would close a cycle is refused by the names it closes on", "[rigid_motion][configuration]")
{
    const std::filesystem::path where = authored("cycle.xml", three("tip", "base", "upper"));

    const expected<frame_window::settings, config::error> read = read_back(where);
    REQUIRE_FALSE(read.has_value());
    REQUIRE(read.error().code == config::error_code::rejected_content);
    REQUIRE(read.error().message.find("tip") != std::string::npos);
    REQUIRE(read.error().message.find("upper") != std::string::npos);
}

TEST_CASE("a document carrying a key the arrangement does not declare is refused, and every value falls back", "[rigid_motion][configuration]")
{
    const std::filesystem::path where = scratch() / "undeclared-key.xml";
    {
        std::ofstream out(where, std::ios::binary | std::ios::trunc);
        out << "<probe><arrangement world_frame=\"upper\">" << three("", "base", "") << "</arrangement></probe>\n";
    }

    const expected<config::document, config::error> read = config::load(described(), config::resolve(where, scratch()));
    REQUIRE_FALSE(read.has_value());
    REQUIRE(read.error().code == config::error_code::rejected_content);
    REQUIRE(read.error().message.find("world_frame") != std::string::npos);

    // The refusal is of the whole document, so the instances it carries are not reached either.
    const config::outcome answered = config::load_or_defaults(described(), config::resolve(where, scratch()), config::expectation::partial);

    REQUIRE(answered.failure.has_value());
    REQUIRE(answered.values.identities(std::string(at) + "/object").empty());
}
