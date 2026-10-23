#include "typed_stage.h"
#include "composed_curve.h"
#include "optimization_keys.h"
#include "scratch_directory.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <algorithm>
#include <filesystem>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr const char *cubic_document = "<optimization><preset name=\"typed\" scenario=\"curve\"/><function source=\"manual\" expression=\"2x^3 + 5b\"/>"
                                       "<parameters><parameter name=\"b\" value=\"1\" lower=\"-5\" upper=\"5\"/></parameters>"
                                       "<range x_lower=\"-3\" x_upper=\"3\"/><seed x=\"1\"/><solve limit=\"50\"/></optimization>";

constexpr const char *unread_document = "<optimization><preset name=\"unread\" scenario=\"curve\"/><function source=\"manual\" expression=\"2x^3 +\"/>"
                                        "<range x_lower=\"-3\" x_upper=\"3\"/><seed x=\"1\"/></optimization>";

constexpr const char *minimum_document = "<optimization><preset name=\"minimum\" scenario=\"curve\"/><function source=\"manual\" expression=\"x^4 - 3x^2 + x\"/>"
                                         "<range x_lower=\"-2.5\" x_upper=\"2.5\"/><seed x=\"2\"/><solve limit=\"50\" form=\"minimum\" method=\"newton\"/></optimization>";

config::location written(const std::string &file, const char *body)
{
    const std::filesystem::path directory = shared_scratch_directory() / "optimization-manual";
    std::filesystem::create_directories(directory);
    std::ofstream(directory / file, std::ios::binary | std::ios::trunc) << body << "\n";

    return config::resolve(file, directory);
}

scenario_settings opened_from(const char *body, const std::string &file, std::size_t dimension)
{
    const config::outcome carried = config::load_or_defaults(config::binding{presets::optimization_keyspace(), written(file, body), config::expectation::partial});
    REQUIRE_FALSE(carried.failure.has_value());

    return presets::keys::opened_settings(carried.values, dimension);
}

std::vector<std::shared_ptr<scene::imgui_window>> no_windows(const scenario_window_inputs &)
{
    return {};
}

// A Manual document composed with no windows, torn down when the case ends.
struct document_stage
{
    scheduler::scheduler loop;
    staged built;
    opened taken;

    document_stage(const char *body, const std::string &file)
            : loop(scheduler::inline_workers)
            , built(loop)
            , taken(open_composing(built.site, {}, baseline(), opened_from(body, file, 1u), &no_windows))
    {
    }

    ~document_stage()
    {
        if(taken.composed != nullptr)
            taken.composed->tear_down();
    }
};

std::vector<Eigen::VectorXd> solved_at;
std::vector<expected<Eigen::MatrixXd, refusal>> solved_slopes;

expected<Eigen::VectorXd, refusal> recorded_solve(const problem &posed, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    const expected<Eigen::VectorXd, refusal> answered = baseline().solver.solve(posed, x0, max_iterations, iterates);
    solved_at                                         = iterates;
    solved_slopes.clear();
    for(const Eigen::VectorXd &x : iterates)
        solved_slopes.push_back(posed.jacobian(x));

    return answered;
}

void check_cubic_slopes(const std::vector<Eigen::VectorXd> &iterates, const std::vector<expected<Eigen::MatrixXd, refusal>> &slopes)
{
    REQUIRE(iterates.size() >= 2u);
    REQUIRE(slopes.size() == iterates.size());
    for(std::size_t k = 0; k < iterates.size(); ++k)
    {
        REQUIRE(slopes[k].has_value());
        CHECK(within((*slopes[k])(0, 0), 6.0 * iterates[k][0] * iterates[k][0]));
    }
}

}

TEST_CASE("a Manual document composes and solves its typed function", "[optimization][typed]")
{
    solved_at.clear();
    const auto registry = std::make_shared<scene::preset_registry>();
    const capabilities recorded{.solver = solver_ops{.solve = &recorded_solve}, .minimizer = baseline().minimizer};
    REQUIRE(presets::register_optimization(registry, std::vector<config::location>{written("registered.xml", cubic_document)}, nullptr, nullptr, recorded, {}, {}, false).size() == 1u);

    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const std::shared_ptr<scene::preset> composed = registry->load_preset("typed")(built.site);
    REQUIRE(composed != nullptr);
    REQUIRE(composed->initialize().has_value());
    REQUIRE(loop.drain().has_value());

    check_cubic_slopes(solved_at, solved_slopes);
    CHECK(std::abs(solved_at.back()[0] - std::cbrt(-2.5)) < 1.0e-12);
    composed->tear_down();
    REQUIRE(loop.retire_strand(composed->work, std::move(composed->release_cb)).has_value());
    REQUIRE(loop.drain().has_value());
}

TEST_CASE("a Manual document publishes the run and the samples of its typed function", "[optimization][typed]")
{
    document_stage stage(cubic_document, "published.xml");
    const std::shared_ptr<const scenario_snapshot> shown = settled(stage.loop, stage.taken);
    REQUIRE(shown != nullptr);

    CHECK_FALSE(shown->run.refused.has_value());
    REQUIRE(shown->run.answer.has_value());
    CHECK(std::abs((*shown->run.answer)[0] - std::cbrt(-2.5)) < 1.0e-12);
    check_cubic_slopes(shown->run.iterates, shown->run.jacobians);
    REQUIRE_FALSE(shown->abscissa.empty());
    for(std::size_t at = 0; at < shown->abscissa.size(); ++at)
        CHECK(within(shown->ordinate[at], 2.0 * std::pow(shown->abscissa[at], 3.0) + 5.0));
}

TEST_CASE("a Manual document whose text does not read composes, publishes nothing and keeps its text", "[optimization][typed]")
{
    document_stage stage(unread_document, "unread.xml");

    CHECK(settled(stage.loop, stage.taken) == nullptr);
    CHECK(stage.taken.inputs->opened.typed.source == function_source::manual);
    CHECK(stage.taken.inputs->opened.typed.expression == "2x^3 +");
}

TEST_CASE("a Manual document in the minimum form runs Newton with the typed function's Hessian", "[optimization][typed]")
{
    document_stage stage(minimum_document, "minimum.xml");
    const std::shared_ptr<const scenario_snapshot> shown = settled(stage.loop, stage.taken);
    REQUIRE(shown != nullptr);

    CHECK(shown->run.minimized_by == minimizer_slot::newton);
    CHECK_FALSE(shown->run.refused.has_value());
    REQUIRE(shown->run.iterates.size() >= 2u);
    REQUIRE(shown->run.hessians.size() == shown->run.iterates.size());
    for(std::size_t k = 0; k < shown->run.iterates.size(); ++k)
    {
        const double x = shown->run.iterates[k][0];
        REQUIRE(shown->run.hessians[k].has_value());
        REQUIRE((shown->run.hessians[k]->rows() == 1 && shown->run.hessians[k]->cols() == 1));
        CHECK(within((*shown->run.hessians[k])(0, 0), 12.0 * x * x - 6.0));
    }
}
