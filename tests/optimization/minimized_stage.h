#ifndef HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_MINIMIZED_STAGE_H
#define HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_MINIMIZED_STAGE_H

#include "composed_curve.h"

#include "panel_keys.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/solve_window.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

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
#include <utility>
#include <algorithm>
#include <type_traits>

namespace praxis::fixture {

inline constexpr const char *quartic_name = "x^4 - 3x^2 + x";

inline double quartic(double x)
{
    return x * x * x * x - 3.0 * x * x + x;
}

inline double quartic_slope(double x)
{
    return 4.0 * x * x * x - 6.0 * x + 1.0;
}

inline double quartic_curvature(double x)
{
    return 12.0 * x * x - 6.0;
}

inline registered_curve quartic_curve()
{
    return registered_curve(quartic_name, &quartic, &quartic_slope, &quartic_curvature);
}

// f = x^4/4 - x^2/2, whose stationary points are a maximum at 0 and minima at -1 and 1.
inline registered_curve double_well()
{
    return registered_curve("well", [](double x) { return x * x * x * x / 4.0 - x * x / 2.0; }, [](double x) { return x * x * x - x; }, [](double x) { return 3.0 * x * x - 1.0; });
}

inline scenario_settings minimum_from(std::string selected, Eigen::VectorXd seed, minimizer_slot method, double step_length, std::uint32_t limit)
{
    std::vector<interval> ranges(static_cast<std::size_t>(seed.size()), interval{-2.5, 2.5});

    return scenario_settings{std::move(selected), std::move(ranges), std::move(seed), limit, 0u, objective_settings{problem_form::minimum, method, step_length}, {}};
}

inline scenario_settings minimum_from(std::string selected, double seed, minimizer_slot method, double step_length, std::uint32_t limit)
{
    return minimum_from(std::move(selected), Eigen::VectorXd::Constant(1, seed), method, step_length, limit);
}

// A Solve window over one composition, whose posted requests a drain runs inline.
struct solve_stage
{
    scheduler::scheduler loop;
    staged built;
    std::shared_ptr<solve_window> panel;
    opened taken;

    template<typename registered>
    solve_stage(registered function, const scenario_settings &settings)
            : loop(scheduler::inline_workers)
            , built(loop)
            , panel()
            , taken()
    {
        const scenario_window_composer kept = [this](const scenario_window_inputs &offered)
        {
            taken.inputs = offered;
            panel        = std::make_shared<solve_window>("Solve", offered);

            return std::vector<std::shared_ptr<scene::imgui_window>>{panel};
        };
        if constexpr(std::is_same_v<registered, registered_curve>)
            taken.composed = compose_curve_scenario(built.site, {std::move(function)}, baseline(), settings, scenario_composition{kept, false});
        else
            taken.composed = compose_surface_scenario(built.site, {std::move(function)}, baseline(), settings, scenario_composition{kept, false});
        REQUIRE(settled(loop, taken) != nullptr);
    }

    drawing draw() const
    {
        return [shown = panel] { shown->render(); };
    }

    std::shared_ptr<const scenario_snapshot> drained()
    {
        REQUIRE(loop.drain().has_value());

        return taken.inputs->seen.read();
    }
};

inline bool descended_by(const scenario_run &run, double alpha)
{
    for(std::size_t k = 0; k + 1 < run.iterates.size(); ++k)
    {
        const double x        = run.iterates[k][0];
        const double expected = x - alpha * quartic_slope(x);
        if(std::abs(run.iterates[k + 1][0] - expected) > 4.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, std::abs(expected)))
            return false;
    }

    return true;
}

}

#endif
