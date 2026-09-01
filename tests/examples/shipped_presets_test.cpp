#include "demo_machine.h"
#include "demo_configuration.h"

#include "praxis/config/store.h"
#include "praxis/config/document.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <filesystem>
#include <string_view>

// The documents the demonstration ships are read from beside its sources, which is where the
// application resolves them from too.
#ifndef PRAXIS_SHIPPED_MACHINE_DIR
    #define PRAXIS_SHIPPED_MACHINE_DIR ""
#endif

using namespace praxis;

namespace {

constexpr const char *demonstration_document = "praxis-manipulator.xml";

// A preset document as the application reads it: the name it is filed under and the values that
// were answered.
struct shipped
{
    std::string file;
    config::document values;
};

// Every document beside the demonstration's own, read against the declaration the application reads
// it with. A directory iteration answers in no particular order, so the result is sorted before any
// caller compares it against anything.
std::vector<shipped> shipped_presets()
{
    REQUIRE(std::filesystem::is_directory(PRAXIS_SHIPPED_MACHINE_DIR));

    std::vector<shipped> read;
    for(const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(PRAXIS_SHIPPED_MACHINE_DIR))
    {
        if(entry.path().extension() != ".xml" || entry.path().filename() == demonstration_document)
            continue;

        INFO(entry.path().filename().string());
        const config::outcome answered = config::load_or_defaults(demo::machine_keyspace(), config::resolve(entry.path(), entry.path().parent_path()), config::expectation::partial);
        REQUIRE_FALSE(answered.failure.has_value());
        read.push_back(shipped{entry.path().filename().string(), answered.values});
    }

    std::ranges::sort(read, {}, &shipped::file);

    return read;
}

// The values a preset opens the arm at. A package root is not needed to read them, so none is given.
presets::arm_scenario opened(const config::document &values)
{
    return demo::read_machine(values, std::filesystem::path());
}

// The scenarios that need the arm somewhere in particular. The two that solve open away from the
// configuration where the wrist axes stand collinear, since a target taken there has a continuum of
// answers rather than several; the velocity one opens where both blocks of its Jacobian are well
// conditioned, so the two drawn bodies have an extent to be seen at. Every other scenario opens at
// the arm's own home, so two documents over one machine show it alike.
bool opens_placed(const config::document &values)
{
    const std::string_view scenario = demo::arm_scenario_labels()[demo::preset_scenario(values)];

    return scenario == "numerical inverse kinematics" || scenario == "analytic inverse kinematics" || scenario == "velocity kinematics";
}

std::vector<std::string> named_documents(const config::document &values)
{
    std::vector<std::string> named;
    for(const std::string &key : demo::preset_keys(values))
    {
        const expected<std::string, config::error> addressed = values.key(demo::preset_instances, key, demo::document_leaf);
        REQUIRE(addressed.has_value());

        const expected<std::string, config::error> document = values.text(addressed.value());
        REQUIRE(document.has_value());
        named.push_back(document.value());
    }

    return named;
}

}

TEST_CASE("every shipped preset document loads against the declaration the application reads it with", "[examples][documents]")
{
    CHECK(shipped_presets().size() == 12u);
}

TEST_CASE("the names the shipped preset documents state are the ones the demonstration offers", "[examples][documents]")
{
    std::vector<std::string> stated;
    for(const shipped &document : shipped_presets())
        stated.push_back(demo::preset_name(document.values));

    std::ranges::sort(stated);

    CHECK(stated ==
          std::vector<std::string>{"kr6r", "kr6r: analytic inverse kinematics", "kr6r: forward kinematics", "kr6r: numerical inverse kinematics", "kr6r: path comparison",
                                   "kr6r: point to point motion", "kr6r: screw chain preview", "kr6r: velocity kinematics", "kr6r: via point motion", "ur3e: forward kinematics",
                                   "ur3e: numerical inverse kinematics", "ur3e: screw chain preview"});
}

TEST_CASE("every shipped preset document names a description of its own", "[examples][documents]")
{
    for(const shipped &document : shipped_presets())
    {
        INFO(document.file);
        CHECK_FALSE(demo::machine_description(document.values).empty());
    }
}

// A spelling outside the sealed set is a refusal rather than a fallback, and a refused document
// never reaches here, so what is left to say is that each document states a scenario at all instead
// of inheriting the first.
TEST_CASE("every shipped preset document states a scenario the declaration admits", "[examples][documents]")
{
    for(const shipped &document : shipped_presets())
    {
        INFO(document.file);
        CHECK(document.values.origin_of("preset/scenario").kind == config::origin_kind::source);
        CHECK(demo::preset_scenario(document.values) < demo::arm_scenario_labels().size());
    }
}

TEST_CASE("each shipped preset opens where its scenario needs it", "[examples][documents]")
{
    std::set<std::string> standing;
    std::set<std::string> placed;
    for(const shipped &document : shipped_presets())
    {
        if(!opened(document.values).initial.isZero())
            standing.insert(document.file);
        if(opens_placed(document.values))
            placed.insert(document.file);
    }

    REQUIRE_FALSE(placed.empty());
    CHECK(standing == placed);
}

TEST_CASE("the demonstration document names exactly the preset documents that are beside it", "[examples][documents]")
{
    const config::outcome answered = config::load_or_defaults(demo::demonstration_keyspace(),
                                                              config::resolve(std::filesystem::path(PRAXIS_SHIPPED_MACHINE_DIR) / demonstration_document, PRAXIS_SHIPPED_MACHINE_DIR));
    REQUIRE_FALSE(answered.failure.has_value());

    std::vector<std::string> named = named_documents(answered.values);
    std::ranges::sort(named);

    std::vector<std::string> present;
    for(const shipped &document : shipped_presets())
        present.push_back(document.file);

    CHECK(named == present);
}
