#include "drawn_graph.h"
#include "composed_curve.h"
#include "minimized_stage.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/plot_window.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;

namespace {

struct quadratic
{
    double x;
    double f;
    double slope;
    double curvature;

    double operator()(double at) const
    {
        return f + slope * (at - x) + 0.5 * curvature * (at - x) * (at - x);
    }
};

bool near(double drawn, double expected)
{
    return std::abs(drawn - expected) <= 1.0e-12 * std::max(1.0, std::abs(expected));
}

// Every finite point of the series lies on the model, and more than one is drawn.
void check_on_model(const scene::plot_series &model, const quadratic &q)
{
    CHECK(model.mark == scene::plot_mark::line);
    REQUIRE(model.abscissa.size() == model.ordinate.size());
    std::size_t finite = 0;
    for(std::size_t at = 0; at < model.abscissa.size(); ++at)
        if(std::isfinite(model.ordinate[at]))
        {
            ++finite;
            CHECK(near(model.ordinate[at], q(model.abscissa[at])));
        }
    CHECK(finite > 1u);
}

// The model's vertex x_k - f'_k / f''_k, marked on the model.
const scene::plot_series &checked_vertex(const scene::plot_frame &shown, const quadratic &q)
{
    const scene::plot_series *vertex = named(shown, "vertex");
    REQUIRE(vertex != nullptr);
    REQUIRE(vertex->abscissa.size() == 1u);
    CHECK(near(vertex->abscissa[0], q.x - q.slope / q.curvature));
    CHECK(near(vertex->ordinate[0], q(vertex->abscissa[0])));

    return *vertex;
}

}

TEST_CASE("a Newton-minimized curve draws the quadratic model at the selected step and marks its vertex", "[optimization][graph][objective]")
{
    graph_stage stage(quartic_curve(), baseline(), minimum_from(quartic_name, 2.0, minimizer_slot::newton, 0.1, 50u));
    const scene::plot_frame shown = stage.at_step(0u);
    REQUIRE(stage.ran->run.iterates.size() > 1u);
    const double next = stage.ran->run.iterates[1][0];
    const quadratic q{2.0, quartic(2.0), quartic_slope(2.0), quartic_curvature(2.0)};

    CHECK(shown.ordinate_label == "f(x)");
    const scene::plot_series *model = named(shown, "quadratic model at x_k");
    REQUIRE(model != nullptr);
    check_on_model(*model, q);

    checked_vertex(shown, q);

    const scene::plot_series *stepped = named(shown, "x_(k+1)");
    REQUIRE(stepped != nullptr);
    CHECK(stepped->mark == scene::plot_mark::segments);
    REQUIRE(stepped->ordinate.size() == 2u);
    CHECK((stepped->abscissa[0] == next && stepped->abscissa[1] == next));
    CHECK(near(stepped->ordinate[0], q(next)));
    CHECK(near(stepped->ordinate[1], quartic(next)));
}

TEST_CASE("a descent-minimized curve draws the tangent at the selected step and the next iterate against it", "[optimization][graph][objective]")
{
    graph_stage stage(quartic_curve(), baseline(), minimum_from(quartic_name, 2.0, minimizer_slot::gradient_descent, 0.05, 50u));
    const scene::plot_frame shown = stage.at_step(0u);
    REQUIRE(stage.ran->run.iterates.size() > 1u);
    const double next = stage.ran->run.iterates[1][0];
    CHECK(near(next, 2.0 - 0.05 * quartic_slope(2.0)));

    const scene::plot_series *line = named(shown, "tangent at x_k");
    REQUIRE(line != nullptr);
    check_on_line(*line, 2.0, quartic(2.0), quartic_slope(2.0));

    const scene::plot_series *stepped = named(shown, "x_(k+1)");
    REQUIRE(stepped != nullptr);
    REQUIRE(stepped->ordinate.size() == 2u);
    CHECK((stepped->abscissa[0] == next && stepped->abscissa[1] == next));
    CHECK(near(stepped->ordinate[0], quartic(2.0) + quartic_slope(2.0) * (next - 2.0)));
    CHECK(near(stepped->ordinate[1], quartic(next)));
    for(const char *name : {"quadratic model at x_k", "vertex", "zero of the tangent"})
        CHECK(named(shown, name) == nullptr);
}

TEST_CASE("a concave step draws the downward model and marks its vertex, where Newton steps", "[optimization][graph][objective]")
{
    graph_stage stage(double_well(), baseline(), minimum_from("well", 0.1, minimizer_slot::newton, 0.1, 50u));
    const scene::plot_frame shown = stage.at_step(0u);
    REQUIRE(stage.ran->run.iterates.size() > 1u);
    const quadratic q{0.1, std::pow(0.1, 4) / 4.0 - 0.1 * 0.1 / 2.0, std::pow(0.1, 3) - 0.1, 3.0 * 0.1 * 0.1 - 1.0};
    REQUIRE(q.curvature < 0.0);

    const scene::plot_series *model = named(shown, "quadratic model at x_k");
    REQUIRE(model != nullptr);
    check_on_model(*model, q);

    const scene::plot_series &vertex = checked_vertex(shown, q);
    CHECK(near(vertex.abscissa[0], stage.ran->run.iterates[1][0]));
    for(const double y : model->ordinate)
        CHECK(!(y > vertex.ordinate[0] + 1.0e-12));
}

TEST_CASE("a Newton run over a curve registered without a second derivative draws its iterate and no model", "[optimization][graph][objective]")
{
    graph_stage stage(registered_curve(quartic_name, &quartic, &quartic_slope), baseline(), minimum_from(quartic_name, 2.0, minimizer_slot::newton, 0.1, 50u));
    const scene::plot_frame shown = stage.at_step(0u);
    REQUIRE(stage.ran->run.iterates.size() == 1u);

    const scene::plot_series *marks = named(shown, "Iterates");
    REQUIRE(marks != nullptr);
    REQUIRE(marks->abscissa.size() == 1u);
    CHECK(marks->abscissa[0] == 2.0);
    CHECK(near(marks->ordinate[0], quartic(2.0)));
    for(const char *name : {"quadratic model at x_k", "vertex", "x_(k+1)", "tangent at x_k"})
        CHECK(named(shown, name) == nullptr);
}

TEST_CASE("a root-form run of the same curve draws the tangent and its zero under g(x)", "[optimization][graph][objective]")
{
    graph_stage stage(quartic_curve(), baseline(), opening(quartic_name));
    const scene::plot_frame shown = stage.at_step(0u);
    CHECK_FALSE(stage.ran->run.minimized_by.has_value());

    CHECK(shown.ordinate_label == "g(x)");
    for(const char *name : {"g(x)", "Iterates", "tangent at x_k", "zero of the tangent", "x_(k+1)"})
        CHECK(named(shown, name) != nullptr);
    for(const char *name : {"f(x)", "quadratic model at x_k", "vertex"})
        CHECK(named(shown, name) == nullptr);
}
