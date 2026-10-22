#include "scratch_directory.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"

#include "praxis/scheduler/strand.h"
#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <utility>
#include <algorithm>
#include <filesystem>

#ifndef PRAXIS_SHIPPED_OPTIMIZATION_DIR
    #define PRAXIS_SHIPPED_OPTIMIZATION_DIR ""
#endif

using namespace praxis;

namespace {

double cubic(double x)
{
    return x * x * x - 2.0 * x - 5.0;
}

double cubic_slope(double x)
{
    return 3.0 * x * x - 2.0;
}

double newton_step(double x)
{
    return x - cubic(x) / cubic_slope(x);
}

std::size_t solves_asked = 0;

expected<Eigen::VectorXd, refusal> counted_solve(const optimization::problem &posed, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    ++solves_asked;

    return optimization::baseline().solver.solve(posed, x0, max_iterations, iterates);
}

// Every x the registered functions were evaluated at, in the order they were asked. The derivative is
// evaluated by the solve and at its iterates, the curve is sampled up to its range's upper end, and the
// Linearization window's point, the middle of the range, is evaluated last.
struct evaluations
{
    std::vector<double> valued;
    std::vector<double> sloped;
};

// Second to last: the run's last iterate or the range's upper end, before the point's evaluation.
double second_to_last(const std::vector<double> &evaluated)
{
    return evaluated.at(evaluated.size() - 2u);
}

optimization::registered_curve recording(const std::shared_ptr<evaluations> &into)
{
    return optimization::registered_curve(
            "x^3 - 2x - 5",
            [into](double x)
            {
                into->valued.push_back(x);
                return cubic(x);
            },
            [into](double x)
            {
                into->sloped.push_back(x);
                return cubic_slope(x);
            });
}

config::location written(const std::string &file, const std::string &body)
{
    const std::filesystem::path directory = fixture::shared_scratch_directory() / "optimization";
    std::filesystem::create_directories(directory);
    std::ofstream(directory / file, std::ios::binary | std::ios::trunc) << body << "\n";

    return config::resolve(file, directory);
}

// What registering over one recording curve answered, and what its first preset composed and ran.
struct opened
{
    std::vector<std::string> named;
    bool composed;
    std::vector<std::string> windows;
    std::shared_ptr<evaluations> seen;
};

void run_composed(const scene::preset_registry::factory &compose, opened &taken)
{
    scheduler::scheduler loop(scheduler::inline_workers);
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    const scene::window_route nowhere            = [](const std::shared_ptr<scene::imgui_window> &) {};
    const scene::preset_site site{*target, loop.main_strand(), *loop.make_strand(), [](std::string) {}, nowhere, nowhere, {}};

    std::shared_ptr<scene::preset> built = compose(site);
    taken.composed                       = built != nullptr;
    if(!taken.composed)
        return;

    for(const std::shared_ptr<scene::imgui_window> &window : built->windows)
        taken.windows.push_back(window->display_name());
    REQUIRE(built->initialize().has_value());
    REQUIRE(loop.drain().has_value());
    built->tear_down();
    REQUIRE(loop.retire_strand(built->work, std::move(built->release_cb)).has_value());
    REQUIRE(loop.drain().has_value());
}

opened open_from(const std::vector<config::location> &documents, bool checks_derivative = false, const optimization::capabilities &solving = optimization::baseline())
{
    opened taken{{}, false, {}, std::make_shared<evaluations>()};
    const std::vector<optimization::registered_curve> curves{recording(taken.seen)};
    const auto registry = std::make_shared<scene::preset_registry>();

    taken.named = presets::register_optimization(registry, documents, nullptr, nullptr, solving, curves, {}, checks_derivative);
    if(!taken.named.empty())
        run_composed(registry->load_preset(taken.named.front()), taken);

    return taken;
}

}

TEST_CASE("a curve document opens the curve scenario and its run is published", "[presets][optimization]")
{
    const opened taken = open_from({config::resolve("curve.xml", PRAXIS_SHIPPED_OPTIMIZATION_DIR)});

    REQUIRE(taken.named == std::vector<std::string>{"Root finding: a cubic"});
    REQUIRE(taken.composed);
    CHECK(taken.windows == std::vector<std::string>{"Function", "Graph", "Display", "Solve", "Linearization"});

    const std::vector<double> &sloped = taken.seen->sloped;
    REQUIRE(sloped.size() >= 4u);
    CHECK(sloped[0] == -2.25);
    CHECK(sloped[1] == newton_step(-2.25));
    CHECK(std::abs(cubic(second_to_last(taken.seen->sloped))) < 1.0e-12);
    CHECK(std::ranges::find(taken.seen->valued, -3.0) != taken.seen->valued.end());
    CHECK(second_to_last(taken.seen->valued) == 3.0);
    CHECK((sloped.back() == 0.0 && taken.seen->valued.back() == 0.0));
}

TEST_CASE("a registration checking derivatives has the point's central difference evaluated", "[presets][optimization]")
{
    const opened taken = open_from({config::resolve("curve.xml", PRAXIS_SHIPPED_OPTIMIZATION_DIR)}, true);

    REQUIRE(taken.composed);
    const double h = std::cbrt(std::numeric_limits<double>::epsilon());
    CHECK(second_to_last(taken.seen->valued) == h);
    CHECK(taken.seen->valued.back() == -h);
}

TEST_CASE("the document's seed, range and iteration limit are the run's", "[presets][optimization]")
{
    const opened taken = open_from({written("limited.xml",
                                            "<optimization><preset name=\"limited\" scenario=\"curve\"/><function name=\"x^3 - 2x - 5\"/>"
                                            "<range x_lower=\"-1\" x_upper=\"2.5\"/><seed x=\"2\"/><solve limit=\"2\"/></optimization>")});

    REQUIRE(taken.composed);
    REQUIRE(taken.seen->sloped.size() >= 2u);
    CHECK(taken.seen->sloped.front() == 2.0);
    CHECK(second_to_last(taken.seen->sloped) == newton_step(newton_step(2.0)));
    CHECK(std::ranges::find(taken.seen->valued, -1.0) != taken.seen->valued.end());
    CHECK(second_to_last(taken.seen->valued) == 2.5);
}

TEST_CASE("a document that leaves the range, seed and limit out opens at the keyspace's defaults", "[presets][optimization]")
{
    const opened taken = open_from({written("bare.xml", "<optimization><preset name=\"bare\" scenario=\"curve\"/><function name=\"x^3 - 2x - 5\"/></optimization>")});

    REQUIRE(taken.composed);
    REQUIRE(taken.seen->sloped.size() >= 2u);
    CHECK(taken.seen->sloped.front() == 0.75);
    CHECK(std::ranges::find(taken.seen->valued, -3.0) != taken.seen->valued.end());
    CHECK(second_to_last(taken.seen->valued) == 3.0);
    CHECK(std::abs(cubic(second_to_last(taken.seen->sloped))) < 1.0e-12);
}

TEST_CASE("a document selecting a function the registration does not carry composes nothing", "[presets][optimization]")
{
    const opened taken = open_from({written("absent.xml", "<optimization><preset name=\"absent\" scenario=\"curve\"/><function name=\"sin(x)\"/></optimization>")});

    CHECK(taken.named == std::vector<std::string>{"absent"});
    CHECK_FALSE(taken.composed);
    CHECK(taken.seen->sloped.empty());
}

TEST_CASE("a document stating a name another one has taken, or none, is left out", "[presets][optimization]")
{
    const config::location first   = written("first.xml", "<optimization><preset name=\"taken\" scenario=\"curve\"/></optimization>");
    const config::location second  = written("second.xml", "<optimization><preset name=\"taken\" scenario=\"curve\"/></optimization>");
    const config::location unnamed = written("unnamed.xml", "<optimization><preset scenario=\"curve\"/></optimization>");
    const config::location missing = config::resolve("missing.xml", fixture::shared_scratch_directory());

    const auto registry = std::make_shared<scene::preset_registry>();
    const std::vector<std::string> named =
            presets::register_optimization(registry, std::vector<config::location>{first, second, unnamed, missing}, nullptr, nullptr, optimization::baseline(), {}, {}, false);

    CHECK(named == std::vector<std::string>{"taken"});
    CHECK(registry->preset_names() == std::vector<std::string>{"taken"});
}

TEST_CASE("a registration handed a solver of its own runs every document's preset with that solver", "[presets][optimization]")
{
    solves_asked = 0;
    const optimization::capabilities counted{.solver = optimization::solver_ops{.solve = &counted_solve}};
    const opened taken = open_from({config::resolve("curve.xml", PRAXIS_SHIPPED_OPTIMIZATION_DIR)}, false, counted);

    REQUIRE(taken.composed);
    CHECK(solves_asked == 1u);
    CHECK(std::abs(cubic(second_to_last(taken.seen->sloped))) < 1.0e-12);
}
