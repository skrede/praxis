#include "panel_keys.h"
#include "imgui_frame.h"
#include "captured_log.h"
#include "panel_labels.h"
#include "minimized_stage.h"
#include "scenario_defaults.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/solve_window.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <span>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <cstddef>
#include <functional>

using namespace praxis;
using namespace praxis::fixture;

namespace {

constexpr std::size_t step_length_row = 3;

// What is logged while the request is made and run on the scenario's strand.
std::string logged_by(solve_stage &stage, const std::function<void()> &asked)
{
    return tests::reported_by(
            [&]
            {
                asked();
                REQUIRE(stage.loop.drain().has_value());
            });
}

}

TEST_CASE("in the minimum form the Solve window offers a method and a step length, and in the root form neither", "[optimization][objective]")
{
    solve_stage minimum(quartic_curve(), minimum_from(quartic_name, 2.0, minimizer_slot::newton, 0.01, 50u));
    solve_stage root(quartic_curve(), scenario_settings{quartic_name, {interval{-2.5, 2.5}}, Eigen::VectorXd::Constant(1, 2.0), 50u, 0u, {}});

    std::size_t offered = 0;
    {
        tests::imgui_frame frames;
        stand_on(frames, minimum.draw(), "Solve", "Method");
        stand_on(frames, minimum.draw(), "Solve", "Step length");
        offered = navigable_items(frames, minimum.draw());
    }
    tests::imgui_frame frames;
    CHECK(offered == navigable_items(frames, root.draw()) + 2u);

    const std::span<const char *const> labels = minimizer_labels();
    REQUIRE(labels.size() == 2u);
    CHECK(std::string(labels[0]) == "Newton");
    CHECK(std::string(labels[1]) == "Gradient descent");
}

TEST_CASE("selecting gradient descent in the Solve window solves again from the seed at the step length", "[optimization][objective]")
{
    solve_stage stage(quartic_curve(), minimum_from(quartic_name, 2.0, minimizer_slot::newton, 0.01, 1000u));
    REQUIRE(stage.taken.inputs->seen.read()->run.minimized_by == minimizer_slot::newton);

    tests::imgui_frame frames;
    take_entry_on(frames, stage.draw(), "Solve", "Method", 1u);
    const std::shared_ptr<const scenario_snapshot> ran = stage.drained();

    CHECK(ran->run.minimized_by == minimizer_slot::gradient_descent);
    REQUIRE(ran->run.iterates.size() >= 3u);
    CHECK(ran->run.iterates.front()[0] == 2.0);
    CHECK(descended_by(ran->run, 0.01));
    CHECK_FALSE(ran->run.refused.has_value());
}

TEST_CASE("a step length left in its field solves again with it", "[optimization][objective]")
{
    solve_stage stage(quartic_curve(), minimum_from(quartic_name, 2.0, minimizer_slot::gradient_descent, 0.02, 1000u));
    tests::imgui_frame frames;
    type_into(frames, stage.draw(), step_length_row, "0.01");
    const std::shared_ptr<const scenario_snapshot> ran = stage.drained();

    REQUIRE(ran->run.iterates.size() >= 2u);
    CHECK(descended_by(ran->run, 0.01));
}

TEST_CASE("a step length that is not finite and positive is logged and changes nothing", "[optimization][objective]")
{
    solve_stage stage(quartic_curve(), minimum_from(quartic_name, 2.0, minimizer_slot::gradient_descent, 0.02, 1000u));
    const std::shared_ptr<const scenario_snapshot> ran = stage.taken.inputs->seen.read();
    tests::imgui_frame frames;

    for(const char *typed : {"0", "-1"})
    {
        CHECK(logged_by(stage, [&] { type_into(frames, stage.draw(), step_length_row, typed); }).find("the last run stays shown") != std::string::npos);
        CHECK(stage.drained() == ran);
    }
    const auto undefined = [&] { command(stage.taken.inputs->scenario, [](scenario_state &state) { state.set_step_length(std::numeric_limits<double>::quiet_NaN()); }); };
    CHECK(logged_by(stage, undefined).find("the last run stays shown") != std::string::npos);
    CHECK(stage.drained() == ran);
}

TEST_CASE("Newton over a curve registered without its second derivative refuses, and gradient descent over it answers", "[optimization][objective]")
{
    solve_stage stage(registered_curve(quartic_name, &quartic, &quartic_slope), minimum_from(quartic_name, 2.0, minimizer_slot::newton, 0.01, 1000u));
    const std::shared_ptr<const scenario_snapshot> refused = stage.taken.inputs->seen.read();

    CHECK(refused->run.refused == refusal::unsupported_input);
    CHECK(refused->run.iterates.size() == 1u);
    CHECK(run_status(*refused) == "Refused: unsupported input. Steps taken: 0.");

    command(stage.taken.inputs->scenario, [](scenario_state &state) { state.select_method(minimizer_slot::gradient_descent); });
    const std::shared_ptr<const scenario_snapshot> answered = stage.drained();
    CHECK_FALSE(answered->run.refused.has_value());
    REQUIRE(answered->run.answer.has_value());
    CHECK(std::abs(quartic_slope((*answered->run.answer)[0])) < curve_objective_tolerance);
    CHECK(answered->run.hessians.empty());
}

TEST_CASE("gradient descent at a step length too large for a surface is refused at the limit with every iterate kept", "[optimization][objective]")
{
    const registered_surface elongated(
            "elongated", [](double x, double y) { return (x * x + 10.0 * y * y) / 2.0; }, [](double x, double y) { return Eigen::Vector2d(x, 10.0 * y); },
            [](double, double) { return Eigen::Matrix2d(Eigen::Vector2d(1.0, 10.0).asDiagonal()); });
    solve_stage stage(elongated, minimum_from("elongated", Eigen::Vector2d(1.0, 1.0), minimizer_slot::gradient_descent, 0.25, 40u));
    const std::shared_ptr<const scenario_snapshot> shown = stage.taken.inputs->seen.read();

    CHECK(shown->run.refused == refusal::no_solution);
    CHECK_FALSE(shown->run.answer.has_value());
    REQUIRE(shown->run.iterates.size() == 41u);
    CHECK(shown->run.objectives.size() == 41u);
    CHECK(std::abs(shown->run.iterates.back()[1]) > 1.0e6);
    CHECK(run_status(*shown) == "Refused: no solution. Steps taken: 40.");
}

TEST_CASE("Newton from a seed near a maximum answers the maximum as converged, and a seed at a stationary point is answered alone", "[optimization][objective]")
{
    solve_stage near(double_well(), minimum_from("well", 0.1, minimizer_slot::newton, 0.01, 50u));
    const std::shared_ptr<const scenario_snapshot> shown = near.taken.inputs->seen.read();

    REQUIRE(shown->run.answer.has_value());
    CHECK(std::abs((*shown->run.answer)[0]) < 1.0e-9);
    CHECK(run_status(*shown) == "Converged. Steps taken: " + std::to_string(shown->run.iterates.size() - 1u) + ".");

    solve_stage at(double_well(), minimum_from("well", 0.0, minimizer_slot::newton, 0.01, 50u));
    const std::shared_ptr<const scenario_snapshot> alone = at.taken.inputs->seen.read();
    CHECK(alone->run.iterates.size() == 1u);
    CHECK(run_status(*alone) == "Converged. Steps taken: 0.");
}
