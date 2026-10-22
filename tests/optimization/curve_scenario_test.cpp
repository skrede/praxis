#include "composed_curve.h"
#include "captured_log.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/optimization/baseline/solver.h"

#include "praxis/scene/plot_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;

namespace {

// The cubic sampled over the held range, ends included, at the values the registration answers.
void check_held_line(const scene::plot_frame &frame)
{
    REQUIRE(frame.abscissa_limits.has_value());
    CHECK(frame.abscissa_limits->lower == -4.0);
    CHECK(frame.abscissa_limits->upper == 4.0);
    REQUIRE(frame.series.size() >= 2u);

    const scene::plot_series &line = frame.series[0];
    CHECK(line.mark == scene::plot_mark::line);
    REQUIRE(line.abscissa.size() == line.ordinate.size());
    REQUIRE(line.abscissa.size() >= 2u);
    CHECK(line.abscissa.front() == -4.0);
    CHECK(line.abscissa.back() == 4.0);
    for(std::size_t at = 0; at < line.abscissa.size(); ++at)
        CHECK(line.ordinate[at] == cubic_value(line.abscissa[at]));
}

void check_marks(const scene::plot_series &marks, const std::vector<Eigen::VectorXd> &solved)
{
    CHECK(marks.mark == scene::plot_mark::markers);
    REQUIRE(marks.abscissa.size() == solved.size());
    REQUIRE(marks.ordinate.size() == solved.size());
    for(std::size_t k = 0; k < solved.size(); ++k)
    {
        CHECK(marks.abscissa[k] == solved[k][0]);
        CHECK(marks.ordinate[k] == cubic_value(solved[k][0]));
    }
}

}

TEST_CASE("a composed curve scenario publishes the bound solver's run and its graph draws it", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);

    const opened taken                                 = open_with(built.site, {cubic()}, baseline(), opening());
    const std::shared_ptr<const scenario_snapshot> ran = settled(loop, taken);

    std::vector<Eigen::VectorXd> solved;
    static_cast<void>(solve(curve_problem(cubic(), 1.0e-10), Eigen::VectorXd::Constant(1, 3.0), 70u, solved));

    REQUIRE(ran != nullptr);
    REQUIRE(solved.size() >= 2u);
    CHECK(ran->run.iterates == solved);
    CHECK(ran->run.iterates.front()[0] == 3.0);
    CHECK_FALSE(ran->run.refused.has_value());

    const scene::plot_reading shown = curve_reading(taken.inputs->seen, scenario_view());
    REQUIRE(shown.frames.size() == 1u);
    check_held_line(shown.frames.front());
    check_marks(shown.frames.front().series[1], solved);

    taken.composed->tear_down();
}

TEST_CASE("a composer returning no windows composes a preset with none that still runs", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);

    const opened taken                                 = open_with(built.site, {cubic()}, baseline(), opening(), false);
    const std::shared_ptr<const scenario_snapshot> ran = settled(loop, taken);

    CHECK(taken.composed->windows.empty());
    REQUIRE(ran != nullptr);
    CHECK(ran->run.iterates.size() >= 2u);
    CHECK(taken.inputs->dimension == 1u);
    CHECK(taken.inputs->registered == std::vector<std::string>{"cubic"});

    taken.composed->tear_down();
}

TEST_CASE("settings naming a function the list does not register compose nothing and name both", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);

    const registered_curve other("other", &cubic_value, &cubic_slope);

    tests::captured_log captured;
    const opened taken = open_with(built.site, {cubic(), other}, baseline(), opening("absent"));

    CHECK(taken.composed == nullptr);
    CHECK_FALSE(taken.inputs.has_value());
    CHECK_THAT(captured.text(), Catch::Matchers::ContainsSubstring("it opens at 'absent', which is not registered (registered: 'cubic', 'other')"));
}

TEST_CASE("a work strand that admits no run composes nothing and says so", "[optimization][scenario]")
{
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    const scene::window_route nowhere            = [](const std::shared_ptr<scene::imgui_window> &) {};
    const scene::preset_site unwired{*target, scheduler::strand{}, scheduler::strand{}, [](std::string) {}, nowhere, nowhere, {}};

    tests::captured_log captured;
    const opened taken = open_with(unwired, {cubic()}, baseline(), opening());

    CHECK(taken.composed == nullptr);
    CHECK_THAT(captured.text(), Catch::Matchers::ContainsSubstring("its work strand admits no run"));
}
