#include "composed_curve.h"
#include "captured_log.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/plot_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;

namespace {

double rootless_value(double x)
{
    return x * x + 1.0;
}

double rootless_slope(double x)
{
    return 2.0 * x;
}

// The cubic from 3 steps to 2.36 and then to about 2.127, so a value undefined below 2.2 is refused
// at the third iterate and nowhere before it.
double cut_value(double x)
{
    return x < 2.2 ? std::numeric_limits<double>::quiet_NaN() : cubic_value(x);
}

// Every sample is kept over the whole range; the ones the cut value refuses stand at NaN.
void check_broken_below_cut(const scene::plot_series &line)
{
    REQUIRE(line.abscissa.size() == line.ordinate.size());
    CHECK(line.abscissa.front() == -4.0);
    for(std::size_t at = 0; at < line.abscissa.size(); ++at)
        CHECK(std::isnan(line.ordinate[at]) == (line.abscissa[at] < 2.2));
}

const scene::plot_series &marks_of(const scene::plot_reading &shown)
{
    REQUIRE(shown.frames.size() == 1u);
    REQUIRE(shown.frames.front().series.size() == 2u);

    return shown.frames.front().series[1];
}

}

TEST_CASE("a solve refused at its limit still publishes every iterate and the refusal", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);

    scenario_settings limited = opening("rootless");
    limited.limit             = 5u;

    const opened taken                                 = open_with(built.site, {registered_curve("rootless", &rootless_value, &rootless_slope)}, baseline(), limited);
    const std::shared_ptr<const scenario_snapshot> ran = settled(loop, taken);

    REQUIRE(ran != nullptr);
    CHECK(ran->run.refused == refusal::no_solution);
    CHECK(ran->run.limit == 5u);
    REQUIRE(ran->run.iterates.size() == 6u);
    CHECK(ran->run.errors.size() == 6u);
    CHECK(ran->run.jacobians.size() == 6u);
    CHECK(marks_of(curve_reading(taken.inputs->seen)).abscissa.size() == 6u);

    taken.composed->tear_down();
}

TEST_CASE("a solve whose error refuses as degenerate publishes the iterates it reached and the refusal", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);

    const opened taken                                 = open_with(built.site, {registered_curve("cubic", &cut_value, &cubic_slope)}, baseline(), opening());
    const std::shared_ptr<const scenario_snapshot> ran = settled(loop, taken);

    REQUIRE(ran != nullptr);
    CHECK(ran->run.refused == refusal::degenerate);
    REQUIRE(ran->run.iterates.size() == 3u);
    CHECK((ran->run.errors[0] && ran->run.errors[1]));
    REQUIRE_FALSE(ran->run.errors[2]);
    CHECK(ran->run.errors[2].error() == refusal::degenerate);

    const scene::plot_reading shown = curve_reading(taken.inputs->seen);
    const scene::plot_series &marks = marks_of(shown);
    REQUIRE(marks.ordinate.size() == 3u);
    CHECK(marks.ordinate[1] == cut_value(ran->run.iterates[1][0]));
    CHECK(std::isnan(marks.ordinate[2]));

    check_broken_below_cut(shown.frames.front().series[0]);

    taken.composed->tear_down();
}

TEST_CASE("an inert bound solver publishes its refusal and the seed alone, and the curve is still drawn", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);

    const opened taken                                 = open_with(built.site, {cubic()}, capabilities{}, opening());
    const std::shared_ptr<const scenario_snapshot> ran = settled(loop, taken);

    REQUIRE(ran != nullptr);
    CHECK(ran->run.refused == refusal::not_implemented);
    REQUIRE(ran->run.iterates.size() == 1u);
    CHECK(ran->run.iterates.front()[0] == 3.0);

    const scene::plot_reading shown = curve_reading(taken.inputs->seen);
    CHECK(marks_of(shown).abscissa == std::vector<double>{3.0});
    CHECK(marks_of(shown).ordinate == std::vector<double>{cubic_value(3.0)});
    CHECK(shown.frames.front().series[0].ordinate.size() == ran->abscissa.size());
    CHECK(ran->abscissa.size() >= 2u);

    taken.composed->tear_down();
}

TEST_CASE("a command through the gate runs again, and one selecting an unregistered function leaves the last run", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);

    const opened taken = open_with(built.site, {cubic()}, baseline(), opening());
    REQUIRE(settled(loop, taken) != nullptr);

    command(taken.inputs->scenario, [](scenario_settings &settings) { settings.seed = -1.0; });
    REQUIRE(loop.drain().has_value());
    const std::shared_ptr<const scenario_snapshot> moved = taken.inputs->seen.read();
    CHECK(moved->run.iterates.front()[0] == -1.0);

    tests::captured_log captured;
    command(taken.inputs->scenario, [](scenario_settings &settings) { settings.selected = "absent"; });
    REQUIRE(loop.drain().has_value());

    CHECK(taken.inputs->seen.read() == moved);
    CHECK_THAT(captured.text(), Catch::Matchers::ContainsSubstring("no function named 'absent' is registered"));

    taken.composed->tear_down();
}

TEST_CASE("the graph says nothing has been evaluated before the first run is published", "[optimization][scenario]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);

    const opened taken = open_with(built.site, {cubic()}, baseline(), opening());

    REQUIRE(taken.inputs.has_value());
    const scene::plot_reading shown = curve_reading(taken.inputs->seen);

    CHECK_FALSE(shown.message.empty());
    CHECK(shown.frames.empty());
}
