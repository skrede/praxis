#include "drawn_graph.h"
#include "composed_curve.h"
#include "minimized_stage.h"
#include "scenario_defaults.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_view.h"
#include "praxis/optimization/scenario_state.h"

#include "praxis/scene/plot_window.h"
#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;

namespace {

scenario_settings from(std::string selected, double seed, std::uint32_t limit)
{
    return scenario_settings{std::move(selected), {interval{-3.0, 3.0}}, Eigen::VectorXd::Constant(1, seed), limit, 0u, {}, {}};
}

// The cubic solved from -2.25, linearized at the range's middle with Show on.
struct toned_cubic : graph_stage
{
    explicit toned_cubic(const registered_curve &curve = cubic(), double seed = -2.25, std::uint32_t limit = 70u)
            : graph_stage(curve, baseline(), from(curve.name, seed, limit))
    {
        taken.inputs->drawn->linearization = true;
        command(taken.inputs->scenario, [](scenario_state &state) { state.linearize(Eigen::VectorXd::Constant(1, 0.0)); });
    }
};

std::optional<std::uint32_t> tone_of(const scene::plot_frame &frame, const std::string &name)
{
    const scene::plot_series *drawn = named(frame, name);
    REQUIRE(drawn != nullptr);

    return drawn->tone;
}

scenario_window_composer graph_only()
{
    return [](const scenario_window_inputs &offered) { return std::vector<std::shared_ptr<scene::imgui_window>>{curve_graph(offered, "Graph")}; };
}

// The graph alone at step 0 with the derivative check on, Show on and p at 0.
scene::plot_frame checked_at_p(const registered_curve &curve, const scenario_settings &settings)
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    opened taken = open_composing(built.site, {curve}, baseline(), settings, graph_only(), true);
    REQUIRE(settled(loop, taken) != nullptr);
    taken.inputs->drawn->linearization = true;
    command(taken.inputs->scenario, [](scenario_state &state) { state.linearize(Eigen::VectorXd::Constant(1, 0.0)); });
    REQUIRE(loop.drain().has_value());
    const scene::plot_frame frame = drawn_now(taken);
    taken.composed->tear_down();
    REQUIRE(named(frame, "central difference at p") != nullptr);

    return frame;
}

// Whether the linearization point p and its tangent may share one tone.
constexpr bool p_and_its_tangent_may_share = true;

void check_pairwise_distinct(const scene::plot_frame &frame)
{
    for(std::size_t at = 0; at < frame.series.size(); ++at)
        for(std::size_t other = at + 1; other < frame.series.size(); ++other)
        {
            const auto [first, second] = std::minmax(frame.series[at].name, frame.series[other].name);
            INFO(first << " and " << second << " share 0x" << std::hex << std::setw(6) << std::setfill('0') << frame.series[at].tone.value_or(0u));
            const bool allowed =
                    (first == "x_(k+1)" && second == "zero of the tangent") || (p_and_its_tangent_may_share && first == "linearization point p" && second == "tangent at p");
            CHECK((allowed || frame.series[at].tone != frame.series[other].tone));
        }
}

}

TEST_CASE("every series the cubic's graph draws wears the tone of what it draws", "[optimization][graph][tone]")
{
    toned_cubic stage;
    REQUIRE_FALSE(stage.ran->run.refused.has_value());
    const scene::plot_frame frame = stage.at_step(0u);
    for(const scene::plot_series &series : frame.series)
    {
        INFO(series.name);
        CHECK(series.tone.has_value());
    }
    CHECK(tone_of(frame, "g(x)") == 0x6495edu);
    CHECK(tone_of(frame, "Iterates") == 0xff4500u);
    CHECK(tone_of(frame, "x_(k+1)") == 0x008080u);
    CHECK(tone_of(frame, "x_k") == 0xff00ffu);
    CHECK(tone_of(frame, "tangent at x_k") == 0xf0e68cu);
    CHECK(tone_of(frame, "zero of the tangent") == 0x008080u);
    CHECK(tone_of(frame, "linearization point p") == 0xffffffu);
    CHECK(tone_of(frame, "tangent at p") == 0xffffffu);
}

TEST_CASE("x_k stands at the selected step's iterate on the curve", "[optimization][graph][tone]")
{
    toned_cubic stage;
    for(const std::uint32_t k : {0u, 2u})
    {
        const scene::plot_frame frame    = stage.at_step(k);
        const scene::plot_series *chosen = named(frame, "x_k");
        REQUIRE(chosen != nullptr);
        const double x = stage.ran->run.iterates[k][0];
        CHECK(chosen->mark == scene::plot_mark::markers);
        CHECK(chosen->abscissa == std::vector<double>{x});
        REQUIRE(chosen->ordinate.size() == 1u);
        CHECK(std::abs(chosen->ordinate[0] - cubic_value(x)) <= 1.0e-12 * std::max(1.0, std::abs(cubic_value(x))));
    }
}

TEST_CASE("a converged root run rings its answer at the last iterate, keyed root", "[optimization][graph][tone]")
{
    toned_cubic stage;
    REQUIRE(stage.ran->run.answer.has_value());
    const scene::plot_frame frame  = stage.at_step(0u);
    const scene::plot_series *ring = named(frame, "root");
    REQUIRE(ring != nullptr);
    CHECK(ring->mark == scene::plot_mark::ring);
    CHECK(ring->tone == 0x00ffffu);
    CHECK(ring->abscissa == std::vector<double>{(*stage.ran->run.answer)[0]});
    CHECK(ring->ordinate == std::vector<double>{named(frame, "Iterates")->ordinate.back()});
}

TEST_CASE("a minimized curve wears the tangent's tones by role under both methods and rings a stationary point", "[optimization][graph][objective][tone]")
{
    graph_stage newton(quartic_curve(), baseline(), minimum_from(quartic_name, 2.0, minimizer_slot::newton, 0.05, 50u));
    REQUIRE_FALSE(newton.ran->run.refused.has_value());
    const scene::plot_frame modeled = newton.at_step(0u);
    CHECK(tone_of(modeled, "quadratic model at x_k") == 0xf0e68cu);
    CHECK(tone_of(modeled, "vertex") == 0x008080u);
    CHECK(tone_of(modeled, "x_(k+1)") == 0x008080u);
    CHECK(tone_of(modeled, "stationary point") == 0x00ffffu);
    CHECK(named(modeled, "root") == nullptr);

    graph_stage descent(quartic_curve(), baseline(), minimum_from(quartic_name, 2.0, minimizer_slot::gradient_descent, 0.05, 50u));
    const scene::plot_frame descended = descent.at_step(0u);
    CHECK(tone_of(descended, "tangent at x_k") == 0xf0e68cu);
    CHECK(tone_of(descended, "x_(k+1)") == 0x008080u);
}

TEST_CASE("a run stopped at its limit or refused rings nothing", "[optimization][graph][tone]")
{
    toned_cubic stopped(cubic(), -2.25, 2u);
    REQUIRE(stopped.ran->run.refused.has_value());
    CHECK(named(stopped.drained(), "root") == nullptr);

    toned_cubic refused(registered_curve("level", [](double x) { return x * x + 1.0; }, [](double x) { return 2.0 * x; }), 0.0);
    REQUIRE(refused.ran->run.refused.has_value());
    CHECK(named(refused.drained(), "root") == nullptr);
}

TEST_CASE("with the iterates switched off neither the iterates, x_k nor the answer's ring is drawn", "[optimization][graph][tone]")
{
    toned_cubic stage;
    scenario_view hidden;
    hidden.iterates                 = false;
    const scene::plot_reading shown = curve_reading(stage.taken.inputs->seen, hidden);
    REQUIRE(shown.frames.size() == 1u);
    for(const char *name : {"Iterates", "x_k", "root"})
        CHECK(named(shown.frames.front(), name) == nullptr);
}

TEST_CASE("the box's ends and the central difference at p wear their tones on the curve", "[optimization][graph][tone]")
{
    toned_cubic boxed(registered_curve("boxed", &cubic_value, &cubic_slope, box{Eigen::VectorXd::Constant(1, -1.0), Eigen::VectorXd::Constant(1, 2.5)}), 0.5);
    CHECK(tone_of(boxed.drained(), "bounds") == 0x4b0082u);

    const scene::plot_frame checked = checked_at_p(registered_curve("cubic", &cubic_value, [](double x) { return -cubic_slope(x); }), opening());
    CHECK(tone_of(checked, "central difference at p") == 0x808000u);
}

TEST_CASE("a converged run with a disagreeing derivative check wears no tone twice outside the allowed pairs", "[optimization][graph][objective][tone][pairs]")
{
    const scene::plot_frame root = checked_at_p(registered_curve("cubic", &cubic_value, [](double x) { return 2.0 * cubic_slope(x); }), opening());
    REQUIRE(named(root, "root") != nullptr);
    check_pairwise_distinct(root);

    const registered_curve doubled(quartic_name, &quartic, [](double x) { return 2.0 * quartic_slope(x); }, &quartic_curvature);
    const scene::plot_frame descended = checked_at_p(doubled, minimum_from(quartic_name, 2.0, minimizer_slot::gradient_descent, 0.05, 50u));
    REQUIRE(named(descended, "stationary point") != nullptr);
    check_pairwise_distinct(descended);
}

TEST_CASE("the ring, the central difference and the floor's selected iterate wear three tones", "[optimization][tone][pairs]")
{
    CHECK(optimization::answer_tone != optimization::central_difference_tone);
    CHECK(optimization::answer_tone != optimization::floor_selected_iterate_tone);
    CHECK(optimization::central_difference_tone != optimization::floor_selected_iterate_tone);
}
