#include "composed_curve.h"
#include "optimization_keys.h"
#include "composed_optimization.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/plot_window.h"

#include "praxis/config/store.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/math/Color.hpp>
#include <threepp/core/Object3D.hpp>

#include <threepp/materials/interfaces.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>

#ifndef PRAXIS_SHIPPED_OPTIMIZATION_DIR
    #define PRAXIS_SHIPPED_OPTIMIZATION_DIR ""
#endif

using namespace praxis;
using namespace praxis::fixture;

namespace {

double shipped_cubic(double x)
{
    return x * x * x - 2.0 * x - 5.0;
}

double shipped_quartic(double x)
{
    return x * x * x * x - 3.0 * x * x + x;
}

std::vector<registered_curve> shipped_curves()
{
    return {registered_curve("x^3 - 2x - 5", &shipped_cubic, [](double x) { return 3.0 * x * x - 2.0; }),
            registered_curve("x^4 - 3x^2 + x", &shipped_quartic, [](double x) { return 4.0 * x * x * x - 6.0 * x + 1.0; }, [](double x) { return 12.0 * x * x - 6.0; })};
}

std::vector<registered_surface> shipped_surfaces()
{
    return {registered_surface(
                    "x^2 + y^2 - 1", [](double x, double y) { return x * x + y * y - 1.0; }, [](double x, double y) { return Eigen::Vector2d(2.0 * x, 2.0 * y); }),
            registered_surface(
                    "x y - 1", [](double x, double y) { return x * y - 1.0; }, [](double x, double y) { return Eigen::Vector2d(y, x); }),
            registered_surface(
                    "(x^2 + 10y^2)/2", [](double x, double y) { return 0.5 * (x * x + 10.0 * y * y); }, [](double x, double y) { return Eigen::Vector2d(x, 10.0 * y); },
                    [](double, double) { return Eigen::Matrix2d(Eigen::Vector2d(1.0, 10.0).asDiagonal()); })};
}

const scene::plot_series &named_in(const scene::plot_frame &frame, const char *name)
{
    const auto found = std::find_if(frame.series.begin(), frame.series.end(), [&name](const scene::plot_series &series) { return series.name == name; });
    REQUIRE(found != frame.series.end());

    return *found;
}

// The shipped minimized quartic as its document reads, re-solved by the method given and drawn at step 0.
scene::plot_frame shipped_quartic_at_step_zero(minimizer_slot method)
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    const config::document read = read_optimization(config::resolve("curve-minimum.xml", PRAXIS_SHIPPED_OPTIMIZATION_DIR));
    const opened taken          = open_with(built.site, shipped_curves(), baseline(), presets::keys::opened_settings(read, 1u));
    REQUIRE(settled(loop, taken) != nullptr);
    command(taken.inputs->scenario, [method](scenario_state &state) { state.select_method(method); });
    command(taken.inputs->scenario, [](scenario_state &state) { state.select_step(0u); });
    REQUIRE(loop.drain().has_value());
    REQUIRE(taken.inputs->seen.read()->run.minimized_by == method);

    const scene::plot_reading shown = curve_reading(taken.inputs->seen, *taken.inputs->drawn);
    taken.composed->tear_down();
    REQUIRE(shown.frames.size() == 1u);

    return shown.frames.front();
}

// The interval the finite ordinates of f and of the iterates span within the held abscissa.
interval spanned_by_f(const scene::plot_frame &frame)
{
    REQUIRE(frame.abscissa_limits.has_value());
    interval span{std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
    for(const char *name : {"f(x)", "Iterates"})
    {
        const scene::plot_series &series = named_in(frame, name);
        for(std::size_t at = 0; at < series.abscissa.size() && at < series.ordinate.size(); ++at)
            if(series.abscissa[at] >= frame.abscissa_limits->lower && series.abscissa[at] <= frame.abscissa_limits->upper && std::isfinite(series.ordinate[at]))
                span = interval{std::min(span.lower, series.ordinate[at]), std::max(span.upper, series.ordinate[at])};
    }
    REQUIRE(span.lower <= span.upper);

    return span;
}

void check_drawn_within_f(const scene::plot_frame &frame)
{
    const interval kept = spanned_by_f(frame);
    for(const scene::plot_series &series : frame.series)
        for(std::size_t at = 0; at < series.abscissa.size() && at < series.ordinate.size(); ++at)
        {
            const double x = series.abscissa[at];
            const double y = series.ordinate[at];
            if(x < frame.abscissa_limits->lower || x > frame.abscissa_limits->upper || !std::isfinite(y))
                continue;
            INFO(series.name << " at x = " << x << " is drawn at " << y << " against f's span [" << kept.lower << ", " << kept.upper << "]");
            CHECK(y >= kept.lower - 1.0e-9 * std::max(1.0, std::abs(kept.lower)));
            CHECK(y <= kept.upper + 1.0e-9 * std::max(1.0, std::abs(kept.upper)));
        }

    const scene::plot_series &stepped = named_in(frame, "x_(k+1)");
    REQUIRE(stepped.ordinate.size() == 2u);
    CHECK(std::abs(stepped.ordinate[1] - shipped_quartic(stepped.abscissa[1])) <= 1.0e-9 * std::max(1.0, std::abs(stepped.ordinate[1])));
}

threepp::Color line_tone(threepp::Scene &scene, const std::string &name)
{
    threepp::Object3D *drawn = scene.getObjectByName(name);
    REQUIRE(drawn != nullptr);
    const auto *material = dynamic_cast<const threepp::MaterialWithColor *>(drawn->material().get());
    REQUIRE(material != nullptr);

    return material->color;
}

}

TEST_CASE("the shipped quartic descended from its seed draws nothing at step 0 outside the span of f and its iterates", "[presets][optimization][graph]")
{
    check_drawn_within_f(shipped_quartic_at_step_zero(minimizer_slot::gradient_descent));
}

TEST_CASE("the shipped quartic minimized by Newton draws nothing at step 0 outside the span of f and its iterates", "[presets][optimization][graph]")
{
    check_drawn_within_f(shipped_quartic_at_step_zero(minimizer_slot::newton));
}

TEST_CASE("the shipped paraboloid draws x_(k+1) at step 0 in the tone of the tangent's zero", "[presets][optimization][surface]")
{
    const composed_optimization composed(config::resolve("surface.xml", PRAXIS_SHIPPED_OPTIMIZATION_DIR), {}, "Root finding: a paraboloid", shipped_surfaces());
    composed.preset->stencil->render();

    CHECK(line_tone(*composed.scene, "x_(k+1)") == threepp::Color(0x008080u));
}
