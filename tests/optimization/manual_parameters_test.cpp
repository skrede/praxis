#include "typed_stage.h"
#include "composed_curve.h"
#include "scenario_defaults.h"
#include "scenario_admission.h"

#include "captured_log.h"

#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/extension/refusal.h"

#include <catch2/catch_test_macros.hpp>

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

const typed_parameter b_at_one{'b', 1.0, interval{-5.0, 5.0}};

Eigen::VectorXd answered(const std::shared_ptr<const scenario_snapshot> &shown)
{
    REQUIRE(shown != nullptr);
    REQUIRE_FALSE(shown->run.refused.has_value());
    REQUIRE(shown->run.answer.has_value());

    return *shown->run.answer;
}

double answered_x(const std::shared_ptr<const scenario_snapshot> &shown)
{
    return answered(shown)[0];
}

}

TEST_CASE("a letter new to the typed text takes the default value and range, and a letter it no longer uses is dropped", "[optimization][typed]")
{
    const std::vector<typed_parameter> kept = parameters_for("bc", {typed_parameter{'b', -2.0, interval{-10.0, 10.0}}, typed_parameter{'d', 4.0, interval{0.0, 8.0}}});
    REQUIRE(kept.size() == 2u);
    CHECK((kept[0].name == 'b' && kept[0].value == -2.0 && kept[0].range.lower == -10.0 && kept[0].range.upper == 10.0));
    CHECK((kept[1].name == 'c' && kept[1].value == parameter_value && kept[1].range.lower == parameter_lower && kept[1].range.upper == parameter_upper));

    typed_stage stage(typed_curve("2x^3 + 5b", 1.0, {b_at_one}));
    CHECK(std::abs(answered_x(stage.asked([](scenario_state &state) { state.set_expression("2x^3 + 5b + c"); })) - std::cbrt(-3.0)) < 1.0e-12);

    const std::shared_ptr<const scenario_snapshot> cubed = stage.asked([](scenario_state &state) { state.set_expression("2x^3"); });
    CHECK(std::abs(2.0 * std::pow(answered_x(cubed), 3.0)) < curve_tolerance);
    CHECK(stage.asked([](scenario_state &state) { state.set_parameter('b', 2.0); }) == cubed);
    CHECK(stage.asked([](scenario_state &state) { state.set_parameter('c', 2.0); }) == cubed);
}

TEST_CASE("a typed text that does not read is logged at the character the field marks and changes nothing", "[optimization][typed]")
{
    typed_stage stage(typed_curve("2x^3 + 5b", 1.0, {b_at_one}));
    const std::shared_ptr<const scenario_snapshot> before = stage.shown();

    const std::string said = tests::reported_by([&stage, &before] { CHECK(stage.asked([](scenario_state &state) { state.set_expression("2x^3 +"); }) == before); });
    CHECK(said.find("an operand was expected at character 7") != std::string::npos);
    CHECK(std::abs(answered_x(stage.asked([](scenario_state &state) { state.set_parameter('b', -2.0); })) - std::cbrt(5.0)) < 1.0e-12);
}

TEST_CASE("a parameter's value or range edit solves again from the seed, and an inadmissible one changes nothing", "[optimization][typed]")
{
    typed_stage stage(typed_curve("2x^3 + 5b", 1.0, {b_at_one}));

    const std::shared_ptr<const scenario_snapshot> moved = stage.asked([](scenario_state &state) { state.set_parameter('b', -2.0); });
    CHECK(std::abs(answered_x(moved) - std::cbrt(5.0)) < 1.0e-12);
    CHECK(moved->run.iterates.front()[0] == 1.0);

    const std::shared_ptr<const scenario_snapshot> widened = stage.asked([](scenario_state &state) { state.set_parameter_range('b', interval{-10.0, 10.0}); });
    CHECK(widened != moved);
    CHECK(answered_x(widened) == answered_x(moved));

    CHECK(stage.asked([](scenario_state &state) { state.set_parameter_range('b', interval{3.0, 3.0}); }) == widened);
    CHECK(stage.asked([](scenario_state &state) { state.set_parameter_range('b', interval{3.0, 1.0}); }) == widened);
    CHECK(stage.asked([](scenario_state &state) { state.set_parameter_range('b', interval{-std::numeric_limits<double>::infinity(), 1.0}); }) == widened);
    CHECK(stage.asked([](scenario_state &state) { state.set_parameter('b', std::numeric_limits<double>::quiet_NaN()); }) == widened);
    CHECK(stage.asked([](scenario_state &state) { state.set_parameter('q', 1.0); }) == widened);
}

TEST_CASE("where a typed function is not finite its samples are gaps and a solve reaching such a point refuses degenerate", "[optimization][typed]")
{
    typed_stage stage(typed_curve("log(x)", 3.0));
    const std::shared_ptr<const scenario_snapshot> shown = stage.shown();
    REQUIRE(shown != nullptr);

    REQUIRE_FALSE(shown->abscissa.empty());
    for(std::size_t at = 0; at < shown->abscissa.size(); ++at)
        CHECK(std::isnan(shown->ordinate[at]) == (shown->abscissa[at] <= 0.0));
    CHECK(shown->run.refused == refusal::degenerate);
    REQUIRE(shown->run.iterates.size() >= 2u);
    CHECK(shown->run.iterates.back()[0] < 0.0);
}

TEST_CASE("a typed function's linearization carries no derivative check, and a registered one's does", "[optimization][typed]")
{
    scenario_settings settings = typed_curve("2x^3 + 5b", 1.0, {b_at_one});
    settings.selected          = "cubic";
    typed_stage stage(settings, {cubic()}, true);

    const std::shared_ptr<const scenario_snapshot> typed = stage.asked([](scenario_state &state) { state.linearize(Eigen::VectorXd::Constant(1, 0.5)); });
    REQUIRE(typed->linearization.has_value());
    REQUIRE(typed->linearization->jacobian.has_value());
    CHECK(within((*typed->linearization->jacobian)(0, 0), 1.5));
    CHECK_FALSE(typed->linearization->check.has_value());

    const std::shared_ptr<const scenario_snapshot> registered = stage.asked([](scenario_state &state) { state.select_source(function_source::registered); });
    REQUIRE(registered->linearization.has_value());
    CHECK(registered->linearization->check.has_value());
}

TEST_CASE("a surface's typed text takes x and y as its variables and every other letter as a parameter", "[optimization][typed]")
{
    typed_stage stage(typed_over("x^2 + ay^2 - 1", {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, Eigen::Vector2d(1.5, 0.5)));
    const Eigen::VectorXd circled = answered(stage.shown());
    CHECK(std::abs(circled.squaredNorm() - 1.0) < 1.0e-12);

    const std::shared_ptr<const scenario_snapshot> squeezed = stage.asked([](scenario_state &state) { state.set_parameter('a', 4.0); });
    const Eigen::VectorXd x                                 = answered(squeezed);
    CHECK(std::abs(x[0] * x[0] + 4.0 * x[1] * x[1] - 1.0) < 1.0e-12);
    REQUIRE(squeezed->grid != nullptr);
    CHECK(within(squeezed->grid->heights.front(), 4.0 + 4.0 * 4.0 - 1.0));
    CHECK(stage.asked([](scenario_state &state) { state.set_parameter('y', 1.0); }) == squeezed);
}
