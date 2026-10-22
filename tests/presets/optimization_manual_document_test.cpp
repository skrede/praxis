#include "composed_curve.h"
#include "optimization_keys.h"
#include "scratch_directory.h"
#include "composed_optimization.h"

#include "panel_keys.h"
#include "imgui_frame.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr const char *carried_document = "<optimization><preset name=\"typed\" scenario=\"curve\"/><function source=\"manual\" expression=\"cx^2 + bx - 1\"/>"
                                         "<parameters><parameter name=\"c\" value=\"3\" lower=\"-1\" upper=\"4\"/><parameter name=\"b\" value=\"-2\" lower=\"-6\" upper=\"0\"/>"
                                         "<parameter name=\"B\" value=\"7\"/></parameters><range x_lower=\"-3\" x_upper=\"3\"/><seed x=\"1\"/><solve limit=\"50\"/></optimization>";

constexpr const char *bare_document = "<optimization><preset name=\"bare\" scenario=\"curve\"/><function source=\"manual\" expression=\"ax^2 + b\"/>"
                                      "<range x_lower=\"-3\" x_upper=\"3\"/><seed x=\"1\"/></optimization>";

config::location written(const std::string &file, const char *body)
{
    const std::filesystem::path directory = shared_scratch_directory() / "optimization_manual";
    std::filesystem::create_directories(directory);
    std::ofstream(directory / file, std::ios::binary | std::ios::trunc) << body << "\n";

    return config::resolve(file, directory);
}

// What the composition's windows open at, the parameters settled against the text.
scenario_settings composed_opening(const scenario_settings &settings)
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const opened taken =
            open_composing(built.site, {}, optimization::baseline(), settings, [](const scenario_window_inputs &) { return std::vector<std::shared_ptr<scene::imgui_window>>{}; });
    static_cast<void>(settled(loop, taken));

    return taken.inputs->opened;
}

void check_parameter(const typed_parameter &read, char name, double value, double lower, double upper)
{
    CHECK(read.name == name);
    CHECK(read.value == value);
    CHECK(read.range.lower == lower);
    CHECK(read.range.upper == upper);
}

void check_carried(const scenario_settings &carried)
{
    CHECK(carried.typed.source == function_source::manual);
    CHECK(carried.typed.expression == "cx^2 + bx - 1");
    REQUIRE(carried.typed.parameters.size() == 2u);
    check_parameter(carried.typed.parameters[0], 'b', -2.0, -6.0, 0.0);
    check_parameter(carried.typed.parameters[1], 'c', 3.0, -1.0, 4.0);
}

}

TEST_CASE("a Manual document opens with every parameter it carries named by one lowercase letter, in letter order", "[presets][optimization][documents][typed]")
{
    const scenario_settings read = presets::keys::opened_settings(read_optimization(written("carried.xml", carried_document)), 1u);

    check_carried(read);
    check_carried(composed_opening(read));
}

TEST_CASE("a Manual document carrying no parameter opens with the defaults for the letters its text uses", "[presets][optimization][documents][typed]")
{
    const scenario_settings read = presets::keys::opened_settings(read_optimization(written("bare.xml", bare_document)), 1u);
    CHECK(read.typed.parameters.empty());

    const scenario_settings settled_opening = composed_opening(read);
    REQUIRE(settled_opening.typed.parameters.size() == 2u);
    check_parameter(settled_opening.typed.parameters[0], 'a', 1.0, -5.0, 5.0);
    check_parameter(settled_opening.typed.parameters[1], 'b', 1.0, -5.0, 5.0);
}

TEST_CASE("a Manual document's text and parameters are saved and reopened as they were", "[presets][optimization][documents][typed]")
{
    const config::location at = written("saved.xml", carried_document);
    {
        const composed_optimization first(at, {}, "typed");
        tests::imgui_frame frames;
        type_into(frames, first.draw("Solve"), 0u, "0.5");
        REQUIRE(config::save(presets::optimization_keyspace(), at, first.offered(read_optimization(at))).has_value());
    }

    const config::document reopened = read_optimization(at);
    CHECK(reopened.real("seed/x").value_or(0.0) == 0.5);
    check_carried(presets::keys::opened_settings(reopened, 1u));

    const composed_optimization second(at, {}, "typed");
    CHECK(second.offered(reopened).empty());
}
