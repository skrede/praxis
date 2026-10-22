#include "configuration_keys.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <fstream>
#include <optional>
#include <filesystem>
#include <string_view>

using namespace praxis;

namespace {

config::declaration listed()
{
    config::declaration shape("probe");
    shape.group("objects").collection("objects/object", "name").field("objects/object/scale", config::field_kind::real, "1.0");
    return shape;
}

std::filesystem::path written(const std::string &name, std::string_view body)
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-rigid-motion-configuration-keys";
    std::filesystem::create_directories(directory);
    const std::filesystem::path where = directory / name;
    std::ofstream(where, std::ios::binary | std::ios::trunc) << body;
    return where;
}

// What a lookup addresses, spelled so a failed check shows the instance it answered.
std::string addressed(const std::optional<std::string> &instance)
{
    return instance.value_or("no instance");
}

}

TEST_CASE("an empty name addresses no instance of a collection that holds one without an identity", "[rigid_motion][configuration]")
{
    const std::filesystem::path where                    = written("unnamed-first.xml", "<probe><objects><object/><object name=\"a\"/></objects></probe>\n");
    const expected<config::document, config::error> read = config::load(listed(), config::resolve(where, where.parent_path()));
    REQUIRE(read.has_value());

    CHECK(addressed(rigid_motion::keys::instance_at(read.value(), "objects/object", "")) == "no instance");
    CHECK(addressed(rigid_motion::keys::instance_at(read.value(), "objects/object", "a")) == "objects/object[1]");
    CHECK(addressed(rigid_motion::keys::instance_at(read.value(), "objects/object", "b")) == "no instance");
}
