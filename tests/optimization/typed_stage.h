#ifndef HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_TYPED_STAGE_H
#define HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_TYPED_STAGE_H

#include "composed_curve.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

namespace praxis::fixture {

inline bool within(double given, double wanted)
{
    return std::abs(given - wanted) <= 4.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, std::abs(wanted));
}

inline scenario_settings typed_over(std::string text, std::vector<interval> ranges, Eigen::VectorXd seed, std::vector<typed_parameter> parameters = {})
{
    return scenario_settings{std::string(), std::move(ranges), std::move(seed), 50u, 0u, {}, typed_settings{function_source::manual, std::move(text), std::move(parameters)}};
}

inline scenario_settings typed_curve(std::string text, double seed, std::vector<typed_parameter> parameters = {})
{
    return typed_over(std::move(text), {interval{-3.0, 3.0}}, Eigen::VectorXd::Constant(1, seed), std::move(parameters));
}

// A typed function composed over a registered list, a curve or a surface by the seed's length, whose
// posted requests a drain runs inline.
struct typed_stage
{
    scheduler::scheduler loop;
    staged built;
    opened taken;

    explicit typed_stage(const scenario_settings &settings, std::vector<registered_curve> curves = {}, bool checks_derivative = false)
            : loop(scheduler::inline_workers)
            , built(loop)
            , taken()
    {
        const scenario_window_composer kept = [this](const scenario_window_inputs &offered)
        {
            taken.inputs = offered;

            return std::vector<std::shared_ptr<scene::imgui_window>>{};
        };
        if(settings.seed.size() == 1)
            taken.composed = compose_curve_scenario(built.site, std::move(curves), baseline(), settings, scenario_composition{kept, checks_derivative});
        else
            taken.composed = compose_surface_scenario(built.site, {}, baseline(), settings, scenario_composition{kept, checks_derivative});
        static_cast<void>(settled(loop, taken));
    }

    ~typed_stage()
    {
        if(taken.composed != nullptr)
            taken.composed->tear_down();
    }

    std::shared_ptr<const scenario_snapshot> shown()
    {
        REQUIRE(loop.drain().has_value());

        return taken.inputs->seen.read();
    }

    template<typename operation>
    std::shared_ptr<const scenario_snapshot> asked(operation work)
    {
        command(taken.inputs->scenario, std::move(work));

        return shown();
    }
};

}

#endif
