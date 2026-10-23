#include "scratch_directory.h"

#include "screw_table_keys.h"

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
    shape.group("joints").collection("joints/joint", "index").field("joints/joint/home", config::field_kind::real, "0");
    return shape;
}

// What a lookup addresses, spelled so a failed check shows the instance it answered.
std::string addressed(const std::optional<std::string> &instance)
{
    return instance.value_or("no instance");
}

}

TEST_CASE("an empty name addresses no row of a joint collection that holds one without an identity", "[presets][configuration]")
{
    const std::filesystem::path directory = fixture::shared_scratch_directory() / "screw_table_keys";
    std::filesystem::create_directories(directory);
    const std::filesystem::path where = directory / "unnamed-first.xml";
    std::ofstream(where, std::ios::binary | std::ios::trunc) << "<probe><joints><joint/><joint index=\"1\"/></joints></probe>\n";

    const expected<config::document, config::error> read = config::load(listed(), config::resolve(where, directory));
    REQUIRE(read.has_value());

    CHECK(addressed(presets::keys::instance_at(read.value(), "joints/joint", "")) == "no instance");
    CHECK(addressed(presets::keys::instance_at(read.value(), "joints/joint", "1")) == "joints/joint[1]");
    CHECK(addressed(presets::keys::instance_at(read.value(), "joints/joint", "2")) == "no instance");
}
