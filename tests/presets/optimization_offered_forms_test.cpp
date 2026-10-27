#include "scratch_directory.h"
#include "composed_optimization.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>
#include <fstream>
#include <filesystem>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr const char *root_document = "<optimization><preset name=\"offered\" scenario=\"curve\"/><function name=\"x^3 - 2x - 5\"/><range x_lower=\"-2\" x_upper=\"2\"/>"
                                      "<seed x=\"1.5\"/><solve limit=\"100\" step=\"0\"/></optimization>";

constexpr const char *minimum_document = "<optimization><preset name=\"offered\" scenario=\"curve\"/><function name=\"x^4 - 3x^2 + x\"/><range x_lower=\"-2\" x_upper=\"2\"/>"
                                         "<seed x=\"1.5\"/><solve limit=\"100\" step=\"0\" form=\"minimum\" method=\"gradient_descent\" step_length=\"0.05\"/></optimization>";

std::vector<optimization::registered_curve> three_curves()
{
    using optimization::problem_form;
    return {optimization::registered_curve(
                    "x^3 - 2x - 5", [](double x) { return x * x * x - 2.0 * x - 5.0; }, [](double x) { return 3.0 * x * x - 2.0; })
                    .offered_under({problem_form::root}),
            optimization::registered_curve(
                    "x^4 - 3x^2 + x", [](double x) { return x * x * x * x - 3.0 * x * x + x; }, [](double x) { return 4.0 * x * x * x - 6.0 * x + 1.0; },
                    [](double x) { return 12.0 * x * x - 6.0; })
                    .offered_under({problem_form::minimum}),
            optimization::registered_curve("x^2 + 1", [](double x) { return x * x + 1.0; }, [](double x) { return 2.0 * x; })};
}

config::location written(const std::string &file, const char *body)
{
    const std::filesystem::path directory = shared_scratch_directory() / "optimization_offered_forms";
    std::filesystem::create_directories(directory);
    std::ofstream(directory / file, std::ios::binary | std::ios::trunc) << body << "\n";

    return config::resolve(file, directory);
}

// The document after the second entry of the Function list is taken and saved.
config::document saved_after_second_entry(const config::location &at)
{
    composed_optimization opened(at, three_curves(), "offered");
    tests::imgui_frame frames;
    take_entry_on(frames, opened.draw("Function##curve"), "Function##curve", "Function", 1u);
    REQUIRE(opened.loop.drain().has_value());
    REQUIRE(config::save(presets::optimization_keyspace(), at, opened.offered(read_optimization(at))).has_value());

    return read_optimization(at);
}

}

TEST_CASE("a minimum document lists only the functions offered for minimization, and the one taken is saved and reopens", "[presets][optimization][documents]")
{
    const config::location at       = written("minimum.xml", minimum_document);
    const config::document reopened = saved_after_second_entry(at);
    CHECK(reopened.text("function/name").value_or(std::string()) == "x^2 + 1");
    CHECK(reopened.text("solve/form").value_or(std::string()) == "minimum");

    const composed_optimization second(at, three_curves(), "offered");
    CHECK(second.edits_of("Function##curve", reopened).empty());
}

TEST_CASE("a root document lists only the functions offered for root finding, and the one taken is saved", "[presets][optimization][documents]")
{
    const config::location at       = written("root.xml", root_document);
    const config::document reopened = saved_after_second_entry(at);
    CHECK(reopened.text("function/name").value_or(std::string()) == "x^2 + 1");

    const composed_optimization second(at, three_curves(), "offered");
    CHECK(second.edits_of("Function##curve", reopened).empty());
}
