#include "scratch_directory.h"
#include "composed_optimization.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/registered.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <cmath>
#include <string>
#include <vector>
#include <cstdint>
#include <fstream>
#include <algorithm>
#include <filesystem>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr const char *opening_document = "<optimization><preset name=\"saved\" scenario=\"curve\"/><function name=\"x^3 - 2x - 5\"/><range x_lower=\"-4\" x_upper=\"4\"/>"
                                         "<seed x=\"3\"/><solve limit=\"70\" step=\"0\"/></optimization>";

constexpr const char *refused_seed = "The seed must be finite";

std::vector<optimization::registered_curve> two_curves()
{
    return {optimization::registered_curve(
                    "x^3 - 2x - 5", [](double x) { return x * x * x - 2.0 * x - 5.0; }, [](double x) { return 3.0 * x * x - 2.0; }),
            optimization::registered_curve("cos(x) - x", [](double x) { return std::cos(x) - x; }, [](double x) { return -std::sin(x) - 1.0; })};
}

constexpr const char *minimum_document = "<optimization><preset name=\"descent\" scenario=\"curve\"/><function name=\"x^4 - 3x^2 + x\"/>"
                                         "<range x_lower=\"-2.5\" x_upper=\"2.5\"/><seed x=\"2\"/>"
                                         "<solve limit=\"1000\" step=\"0\" form=\"minimum\" method=\"newton\" step_length=\"0.05\"/></optimization>";

std::vector<optimization::registered_curve> quartic()
{
    return {optimization::registered_curve(
            "x^4 - 3x^2 + x", [](double x) { return x * x * x * x - 3.0 * x * x + x; }, [](double x) { return 4.0 * x * x * x - 6.0 * x + 1.0; },
            [](double x) { return 12.0 * x * x - 6.0; })};
}

config::location written(const std::string &file, const char *body = opening_document)
{
    const std::filesystem::path directory = shared_scratch_directory() / "optimization_saved";
    std::filesystem::create_directories(directory);
    std::ofstream(directory / file, std::ios::binary | std::ios::trunc) << body << "\n";

    return config::resolve(file, directory);
}

void edit_function_and_range(tests::imgui_frame &frames, const drawing &function)
{
    REQUIRE(entry_taken_at(frames, function, 1u));
    type_into(frames, function, 2u, "-3");
    type_into(frames, function, 3u, "3.5");
}

// Run is reached by a step sideways, since the step below the limit field may land on Step once.
void edit_and_run(tests::imgui_frame &frames, const drawing &solve)
{
    type_into(frames, solve, 0u, "0.5");
    type_into(frames, solve, 1u, "40");
    stand_below_top(frames, solve, 2u);
    tap(frames, solve, ImGuiKey_LeftArrow);
    tap(frames, solve, ImGuiKey_Space);
}

// Gradient descent selected, then a step length of 0.02 left in its field and -1 after it, which the
// scenario refuses.
void choose_descent(composed_optimization &opened)
{
    tests::imgui_frame frames;
    take_entry_on(frames, opened.draw("Solve##curve"), "Solve##curve", "Method", 1u);
    REQUIRE(opened.loop.drain().has_value());
    type_into(frames, opened.draw("Solve##curve"), 3u, "0.02");
    REQUIRE(opened.loop.drain().has_value());
    type_into(frames, opened.draw("Solve##curve"), 3u, "-1");
    REQUIRE(opened.loop.drain().has_value());
}

template<typename value>
value read_back(const config::document &values, const char *key, expected<value, config::error> (config::document::*reading)(std::string_view) const)
{
    return (values.*reading)(key).value_or(value());
}

void check_edited(const config::document &reopened)
{
    CHECK(read_back<std::string>(reopened, "function/name", &config::document::text) == "cos(x) - x");
    CHECK(read_back<double>(reopened, "range/x_lower", &config::document::real) == -3.0);
    CHECK(read_back<double>(reopened, "range/x_upper", &config::document::real) == 3.5);
    CHECK(read_back<double>(reopened, "seed/x", &config::document::real) == 0.5);
    CHECK(read_back<std::int64_t>(reopened, "solve/limit", &config::document::integer) == 40);
    CHECK(read_back<std::int64_t>(reopened, "solve/step", &config::document::integer) == 2);
}

}

TEST_CASE("the function, its range, the seed, the limit and the selected step are saved and reopened", "[presets][optimization][documents]")
{
    const config::location at = written("saved.xml");
    {
        composed_optimization first(at, two_curves(), "saved");
        tests::imgui_frame frames;
        edit_function_and_range(frames, first.draw("Function##curve"));
        edit_and_run(frames, first.draw("Solve##curve"));
        REQUIRE(first.loop.drain().has_value());
        tweak_from(frames, first.draw("Solve##curve"), 3u, 2u);
        REQUIRE(first.loop.drain().has_value());

        REQUIRE(config::save(presets::optimization_keyspace(), at, first.offered(read_optimization(at))).has_value());
    }

    const config::document reopened = read_optimization(at);
    check_edited(reopened);

    const composed_optimization second(at, two_curves(), "saved");
    CHECK(second.offered(reopened).empty());
}

TEST_CASE("a scenario opened and left as it opened offers nothing to save", "[presets][optimization][documents]")
{
    const config::location at = written("untouched.xml");
    const composed_optimization opened(at, two_curves(), "saved");

    CHECK(opened.offered(read_optimization(at)).empty());
}

TEST_CASE("the method and the step length the Solve window holds are saved and reopened, and the form is never written", "[presets][optimization][documents]")
{
    const config::location at = written("descent.xml", minimum_document);
    {
        composed_optimization first(at, quartic(), "descent");
        choose_descent(first);

        const std::vector<config::edit> edits = first.offered(read_optimization(at));
        CHECK(std::ranges::none_of(edits, [](const config::edit &made) { return made.key == "solve/form"; }));
        REQUIRE(config::save(presets::optimization_keyspace(), at, edits).has_value());
    }

    const config::document reopened = read_optimization(at);
    CHECK(read_back<std::string>(reopened, "solve/method", &config::document::text) == "gradient_descent");
    CHECK(read_back<double>(reopened, "solve/step_length", &config::document::real) == 0.02);
    CHECK(read_back<std::string>(reopened, "solve/form", &config::document::text) == "minimum");

    const composed_optimization second(at, quartic(), "descent");
    CHECK(second.offered(reopened).empty());
}

TEST_CASE("in the root form the Solve window writes neither a method nor a step length", "[presets][optimization][documents]")
{
    const config::location at = written("rooted.xml");
    const composed_optimization opened(at, two_curves(), "saved");
    tests::imgui_frame frames;
    type_into(frames, opened.draw("Solve##curve"), 0u, "0.5");

    const std::vector<config::edit> edits = opened.offered(read_optimization(at));
    REQUIRE_FALSE(edits.empty());
    CHECK(std::ranges::none_of(edits, [](const config::edit &made) { return made.key.starts_with("solve/method") || made.key.starts_with("solve/step_length"); }));
}

TEST_CASE("a seed typed that is no finite number is shown refused and not offered, and a finite one typed after it is", "[presets][optimization][documents]")
{
    const config::location at = written("refused_seed.xml");
    const composed_optimization opened(at, two_curves(), "saved");
    const auto seed_offered = [&opened, &at]
    { return std::ranges::count_if(opened.edits_of("Solve##curve", read_optimization(at)), [](const config::edit &made) { return made.key == "seed/x"; }); };
    tests::imgui_frame frames;
    for(const char *typed : {"1e999", "inf", "nan"})
    {
        CAPTURE(typed);
        type_into(frames, opened.draw("Solve##curve"), 0u, typed);
        CHECK(seed_offered() == 0);
        CHECK(opened.written(frames, "Solve##curve").find(refused_seed) != std::string::npos);
    }

    type_into(frames, opened.draw("Solve##curve"), 0u, "0.5");
    CHECK(seed_offered() == 1);
    CHECK(opened.written(frames, "Solve##curve").find(refused_seed) == std::string::npos);
}
