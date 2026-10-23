#include "drawn_graph.h"
#include "drawn_surface.h"
#include "composed_curve.h"
#include "minimized_stage.h"
#include "scenario_answers.h"
#include "minimized_surface.h"

#include "imgui_frame.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"
#include "praxis/optimization/linearization_window.h"

#include "praxis/scene/plot_window.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <memory>
#include <string>
#include <vector>

using namespace praxis;
using namespace praxis::fixture;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::StartsWith;

namespace {

// One function composed with the Linearization window, and the Graph beside it where it is a curve.
struct named_stage
{
    scheduler::scheduler loop;
    staged built;
    std::shared_ptr<linearization_window> panel;
    opened taken;

    named_stage(const registered_curve &curve, const scenario_settings &settings, bool checks_derivative)
            : loop(scheduler::inline_workers)
            , built(loop)
            , panel()
            , taken(open_composing(built.site, {curve}, baseline(), settings, composer(panel, true), checks_derivative))
    {
        REQUIRE(settled(loop, taken) != nullptr);
    }

    named_stage(const registered_surface &surface, const scenario_settings &settings)
            : loop(scheduler::inline_workers)
            , built(loop)
            , panel()
            , taken(open_surface_with(built.site, {surface}, baseline(), settings, composer(panel, false)))
    {
        REQUIRE(settled(loop, taken) != nullptr);
    }

    ~named_stage()
    {
        taken.composed->tear_down();
    }

    static scenario_window_composer composer(std::shared_ptr<linearization_window> &kept, bool graphed)
    {
        return [&kept, graphed](const scenario_window_inputs &offered)
        {
            kept = std::make_shared<linearization_window>("Linearization", offered);
            std::vector<std::shared_ptr<scene::imgui_window>> windows{kept};
            if(graphed)
                windows.push_back(curve_graph(offered, "Graph"));

            return windows;
        };
    }

    std::shared_ptr<const scenario_snapshot> shown() const
    {
        const std::shared_ptr<const scenario_snapshot> published = taken.inputs->seen.read();
        REQUIRE(published != nullptr);
        REQUIRE(published->linearization.has_value());

        return published;
    }

    std::vector<std::string> lines(bool checks_derivative) const
    {
        const std::shared_ptr<const scenario_snapshot> published = shown();

        return linearization_lines(*published->linearization, drawn_form(published->run), checks_derivative);
    }

    // What the Linearization window writes when drawn once.
    std::string written() const
    {
        std::string drawn;
        tests::imgui_frame frames;
        frames.draw(
                [this, &drawn]
                {
                    ImGui::LogToBuffer();
                    panel->render();
                    drawn = GImGui->LogBuffer.c_str();
                    ImGui::LogFinish();
                });

        return drawn;
    }
};

scenario_settings quartic_minimum()
{
    return minimum_from(quartic_name, 2.0, minimizer_slot::newton, 0.1, 50u);
}

registered_curve quartic_wrong_by_sign()
{
    return registered_curve(quartic_name, &quartic, [](double x) { return -quartic_slope(x); }, &quartic_curvature);
}

}

TEST_CASE("a minimized curve's Linearization window names the function f and its derivative f', as its Graph does", "[optimization][linearization][objective]")
{
    named_stage staged_curve(quartic_curve(), quartic_minimum(), false);
    REQUIRE(drawn_form(staged_curve.shown()->run) == problem_form::minimum);

    const std::vector<std::string> lines = linearization_lines(*staged_curve.shown()->linearization, problem_form::minimum, false);
    REQUIRE(lines.size() == 2u);
    CHECK_THAT(lines[0], StartsWith("f(x) = "));
    CHECK_THAT(lines[1], StartsWith("f'(x) = "));
    CHECK(staged_curve.lines(false) == lines);

    const std::string written = staged_curve.written();
    CHECK_THAT(written, ContainsSubstring("f(x) = ") && ContainsSubstring("f'(x) = "));
    CHECK_FALSE(written.find("g(x)") != std::string::npos);
    CHECK_FALSE(written.find("g'(x)") != std::string::npos);

    const scene::plot_frame frame = drawn_now(staged_curve.taken);
    CHECK(frame.ordinate_label == "f(x)");
    REQUIRE_FALSE(frame.series.empty());
    CHECK(frame.series.front().name == "f(x)");
}

TEST_CASE("with the check opted in over a minimized curve, the central difference is named for f'", "[optimization][linearization][objective]")
{
    named_stage staged_curve(quartic_wrong_by_sign(), quartic_minimum(), true);

    const std::vector<std::string> lines = staged_curve.lines(true);
    REQUIRE(lines.size() == 4u);
    CHECK_THAT(lines[2], StartsWith("f'(x) by central difference = "));
    CHECK_THAT(lines[3], StartsWith("The derivative handed in"));
}

TEST_CASE("a curve solved for a root keeps the names g and g' in both windows", "[optimization][linearization]")
{
    named_stage staged_curve(cubic(), opening(), false);
    REQUIRE(drawn_form(staged_curve.shown()->run) == problem_form::root);

    const std::vector<std::string> lines = staged_curve.lines(false);
    REQUIRE(lines.size() == 2u);
    CHECK_THAT(lines[0], StartsWith("g(x) = "));
    CHECK_THAT(lines[1], StartsWith("g'(x) = "));
    CHECK_THAT(staged_curve.written(), ContainsSubstring("g(x) = ") && ContainsSubstring("g'(x) = "));
    CHECK(drawn_now(staged_curve.taken).ordinate_label == "g(x)");
}

TEST_CASE("a surface keeps the names f(x, y) and grad f(x, y) in both forms", "[optimization][surface][linearization][objective]")
{
    for(const problem_form form : {problem_form::minimum, problem_form::root})
    {
        named_stage staged_surface(elliptic(), elliptic_minimum(form));
        const std::vector<std::string> lines = staged_surface.lines(false);
        REQUIRE(lines.size() == 2u);
        CHECK_THAT(lines[0], StartsWith("f(x, y) = "));
        CHECK_THAT(lines[1], StartsWith("grad f(x, y) = "));
        CHECK(linearization_lines(*staged_surface.shown()->linearization, form, false) == lines);
    }
}
