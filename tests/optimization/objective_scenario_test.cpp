#include "composed_curve.h"
#include "minimized_stage.h"
#include "optimization_keys.h"
#include "scenario_defaults.h"
#include "scratch_directory.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/solve_window.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/optimization/baseline/minimizer.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <utility>
#include <algorithm>
#include <filesystem>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr const char *minimum_document = "<optimization><preset name=\"minimum\" scenario=\"curve\"/><function name=\"x^4 - 3x^2 + x\"/>"
                                         "<range x_lower=\"-2.5\" x_upper=\"2.5\"/><seed x=\"2\"/><solve limit=\"50\" form=\"minimum\" method=\"newton\"/></optimization>";

config::location written(const std::string &file)
{
    const std::filesystem::path directory = shared_scratch_directory() / "optimization-objective";
    std::filesystem::create_directories(directory);
    std::ofstream(directory / file, std::ios::binary | std::ios::trunc) << minimum_document << "\n";

    return config::resolve(file, directory);
}

config::document read(const config::location &at)
{
    const config::outcome carried = config::load_or_defaults(config::binding{presets::optimization_keyspace(), at, config::expectation::partial});
    REQUIRE_FALSE(carried.failure.has_value());

    return carried.values;
}

std::vector<Eigen::VectorXd> newton_reached;
std::size_t solves_asked = 0;

expected<Eigen::VectorXd, refusal> recorded_newton(const problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    const expected<Eigen::VectorXd, refusal> answered = optimization::newton(p, x0, max_iterations, iterates);
    newton_reached                                    = iterates;

    return answered;
}

expected<Eigen::VectorXd, refusal> counted_solve(const problem &posed, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    ++solves_asked;

    return baseline().solver.solve(posed, x0, max_iterations, iterates);
}

// Composes the preset on a scene of its own, runs what it posted and retires it.
void run_preset(const scene::preset_registry::factory &compose)
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const std::shared_ptr<scene::preset> composed = compose(built.site);
    REQUIRE(composed != nullptr);
    REQUIRE(composed->initialize().has_value());
    REQUIRE(loop.drain().has_value());

    composed->tear_down();
    REQUIRE(loop.retire_strand(composed->work, std::move(composed->release_cb)).has_value());
    REQUIRE(loop.drain().has_value());
}

void check_minimized(const scenario_run &run)
{
    REQUIRE(run.iterates.size() >= 2u);
    CHECK(run.iterates.front()[0] == 2.0);
    CHECK(run.minimized_by == minimizer_slot::newton);
    CHECK_FALSE(run.refused.has_value());
    REQUIRE(run.answer.has_value());
    CHECK(std::abs(quartic_slope((*run.answer)[0])) < curve_objective_tolerance);
    REQUIRE(run.objectives.size() == run.iterates.size());
    REQUIRE(run.gradients.size() == run.iterates.size());
    REQUIRE(run.hessians.size() == run.iterates.size());
    REQUIRE(run.errors.size() == run.iterates.size());
}

}

TEST_CASE("a document in the minimum form runs the bound Newton minimizer over a registered curve", "[optimization][objective]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const scenario_settings settings = presets::keys::opened_settings(read(written("minimum.xml")), 1u);
    const opened taken =
            open_composing(built.site, {quartic_curve()}, baseline(), settings, [](const scenario_window_inputs &) { return std::vector<std::shared_ptr<scene::imgui_window>>{}; });
    const std::shared_ptr<const scenario_snapshot> shown = settled(loop, taken);
    REQUIRE(shown != nullptr);

    check_minimized(shown->run);
    const std::vector<iterate_row> rows = iterate_rows(shown->run);
    REQUIRE(rows.size() == shown->run.iterates.size());
    for(std::size_t k = 0; k < rows.size(); ++k)
    {
        const double x = shown->run.iterates[k][0];
        CHECK(*shown->run.objectives[k] == quartic(x));
        CHECK((*shown->run.gradients[k])[0] == quartic_slope(x));
        CHECK((*shown->run.hessians[k])(0, 0) == quartic_curvature(x));
        REQUIRE(rows[k].cells.size() == 3u);
        CHECK(std::abs(std::stod(rows[k].cells[1]) - quartic(x)) <= 1.0e-10 * std::max(1.0, std::abs(quartic(x))));
    }
}

TEST_CASE("a preset registered from a document in the minimum form hands its run to the bound minimizer and never to the solver", "[optimization][objective]")
{
    solves_asked = 0;
    newton_reached.clear();
    const capabilities recorded{.solver    = solver_ops{.solve = &counted_solve},
                                .minimizer = minimizer_ops{.newton = &recorded_newton, .gradient_descent = &optimization::gradient_descent}};
    const auto registry = std::make_shared<scene::preset_registry>();
    const std::vector<config::location> documents{written("registered.xml")};
    REQUIRE(presets::register_optimization(registry, documents, nullptr, nullptr, recorded, std::vector<registered_curve>{quartic_curve()}, {}, false).size() == 1u);

    run_preset(registry->load_preset("minimum"));

    CHECK(solves_asked == 0u);
    REQUIRE(newton_reached.size() >= 2u);
    CHECK(newton_reached.front()[0] == 2.0);
    CHECK(std::abs(quartic_slope(newton_reached.back()[0])) < curve_objective_tolerance);
}
