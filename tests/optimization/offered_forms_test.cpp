#include "drawn_surface.h"
#include "composed_curve.h"

#include "captured_log.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>

using namespace praxis;
using namespace praxis::fixture;

namespace {

using names = std::vector<std::string>;

scenario_settings in_form(std::string selected, problem_form form)
{
    scenario_settings settings = opening(std::move(selected));
    settings.objective         = objective_settings{form, minimizer_slot::gradient_descent, 0.1};

    return settings;
}

scenario_settings surface_in(std::string selected, problem_form form)
{
    return scenario_settings{
            std::move(selected), {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, Eigen::Vector2d(1.0, 0.5), 50u, 0u, objective_settings{form, minimizer_slot::gradient_descent, 0.1}, {}};
}

registered_curve square()
{
    return registered_curve("x^2", [](double x) { return x * x; }, [](double x) { return 2.0 * x; });
}

registered_curve shifted()
{
    return registered_curve("x - 1", [](double x) { return x - 1.0; }, [](double) { return 1.0; });
}

std::vector<registered_curve> three_curves()
{
    return {cubic().offered_under({problem_form::root}), square().offered_under({problem_form::minimum}), shifted()};
}

std::vector<registered_surface> three_surfaces()
{
    const registered_surface bowl("x^2 + y^2", [](double x, double y) { return x * x + y * y; }, [](double x, double y) { return Eigen::Vector2d(2.0 * x, 2.0 * y); });
    const registered_surface saddle("x y", [](double x, double y) { return x * y; }, [](double x, double y) { return Eigen::Vector2d(y, x); });
    const registered_surface valley("x^2 + 10y^2", [](double x, double y) { return x * x + 10.0 * y * y; }, [](double x, double y) { return Eigen::Vector2d(2.0 * x, 20.0 * y); });

    return {bowl, saddle.offered_under({problem_form::root}), valley.offered_under({problem_form::minimum})};
}

names curves_listed(std::vector<registered_curve> curves, const scenario_settings &settings)
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const opened taken = open_with(built.site, std::move(curves), baseline(), settings, false);
    REQUIRE(settled(loop, taken) != nullptr);
    taken.composed->tear_down();

    return taken.inputs->registered;
}

names surfaces_listed(const scenario_settings &settings)
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const opened taken = open_surface_with(built.site, three_surfaces(), baseline(), settings);
    REQUIRE(settled(loop, taken) != nullptr);
    taken.composed->tear_down();

    return taken.inputs->registered;
}

}

TEST_CASE("a curve scenario lists only the functions offered under the form its settings state, in the order handed", "[optimization][scenario]")
{
    CHECK(curves_listed(three_curves(), in_form("cubic", problem_form::root)) == names{"cubic", "x - 1"});
    CHECK(curves_listed(three_curves(), in_form("x^2", problem_form::minimum)) == names{"x^2", "x - 1"});
}

TEST_CASE("functions registered without naming a form are listed under every form", "[optimization][scenario]")
{
    CHECK(curves_listed({cubic(), shifted()}, in_form("cubic", problem_form::root)) == names{"cubic", "x - 1"});
    CHECK(curves_listed({cubic(), shifted()}, in_form("cubic", problem_form::minimum)) == names{"cubic", "x - 1"});
}

TEST_CASE("a surface scenario lists only the functions offered under the form its settings state", "[optimization][scenario]")
{
    CHECK(surfaces_listed(surface_in("x y", problem_form::root)) == names{"x^2 + y^2", "x y"});
    CHECK(surfaces_listed(surface_in("x^2 + 10y^2", problem_form::minimum)) == names{"x^2 + y^2", "x^2 + 10y^2"});
}

TEST_CASE("settings opening at a function their form does not offer compose nothing and name the form", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);

    tests::captured_log captured;
    const opened taken = open_with(built.site, three_curves(), baseline(), in_form("cubic", problem_form::minimum));

    CHECK(taken.composed == nullptr);
    CHECK_FALSE(taken.inputs.has_value());
    CHECK_THAT(captured.text(), Catch::Matchers::ContainsSubstring("it opens at 'cubic', which is not registered for the minimum form (registered: 'x^2', 'x - 1')"));
}

TEST_CASE("a function is selected by its index in the list offered under the form", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const opened taken = open_with(built.site, three_curves(), baseline(), in_form("x^2", problem_form::minimum), false);
    REQUIRE(settled(loop, taken) != nullptr);

    command(taken.inputs->scenario, [](scenario_state &state) { state.select(1u); });
    REQUIRE(loop.drain().has_value());
    const std::shared_ptr<const scenario_snapshot> drawn = taken.inputs->seen.read();
    REQUIRE(drawn != nullptr);
    CHECK(drawn->selected == 1u);
    REQUIRE(!drawn->abscissa.empty());
    REQUIRE(drawn->abscissa.size() == drawn->ordinate.size());
    for(std::size_t at = 0; at < drawn->abscissa.size(); ++at)
        CHECK(drawn->ordinate[at] == drawn->abscissa[at] - 1.0);

    tests::captured_log captured;
    command(taken.inputs->scenario, [](scenario_state &state) { state.select(2u); });
    REQUIRE(loop.drain().has_value());
    CHECK(taken.inputs->seen.read() == drawn);
    CHECK_THAT(captured.text(), Catch::Matchers::ContainsSubstring("function 3 was selected of the 2 registered with this scenario"));

    taken.composed->tear_down();
}
