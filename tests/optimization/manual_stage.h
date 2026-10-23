#ifndef HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_MANUAL_STAGE_H
#define HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_MANUAL_STAGE_H

#include "composed_curve.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/curve_graph.h"
#include "praxis/optimization/function_window.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace praxis::fixture {

// The Function window over a curve or a surface, by the seed's length, the curve beside its graph,
// whose posted requests a drain runs inline.
struct manual_stage
{
    scheduler::scheduler loop;
    staged built;
    std::shared_ptr<function_window> panel;
    opened taken;

    explicit manual_stage(const scenario_settings &settings, std::vector<registered_curve> curves = {cubic()})
            : loop(scheduler::inline_workers)
            , built(loop)
            , panel()
            , taken()
    {
        const scenario_window_composer kept = [this](const scenario_window_inputs &offered)
        {
            taken.inputs = offered;
            panel        = std::make_shared<function_window>("Function", offered);
            if(offered.dimension == 2u)
                return std::vector<std::shared_ptr<scene::imgui_window>>{panel};

            return std::vector<std::shared_ptr<scene::imgui_window>>{curve_graph(offered, "Graph"), panel};
        };
        if(settings.seed.size() == 1)
            taken.composed = compose_curve_scenario(built.site, std::move(curves), baseline(), settings, scenario_composition{kept, false});
        else
            taken.composed = compose_surface_scenario(built.site, {}, baseline(), settings, scenario_composition{kept, false});
        static_cast<void>(settled(loop, taken));
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

    // What the window writes when drawn once more.
    std::string written(tests::imgui_frame &frames) const
    {
        std::string drawn;
        frames.draw_frame(
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

inline void select_manual(tests::imgui_frame &frames, const drawing &draw)
{
    take_entry_on(frames, draw, "Function", "Source", 1u);
}

// Activating the field selects what it holds, so the characters typed stand in for it unless the
// cursor is first sent to its end; the field is left active.
inline void type_expression(tests::imgui_frame &frames, const drawing &draw, const char *typed, bool appending = false)
{
    stand_on(frames, draw, "Function", "##expression");
    tap(frames, draw, ImGuiKey_Space);
    if(appending)
        tap(frames, draw, ImGuiKey_End);
    for(const char *at = typed; *at != 0; ++at)
    {
        ImGui::GetIO().AddInputCharacter(static_cast<unsigned int>(*at));
        frames.draw_frame(draw);
    }
}

inline void leave_field(tests::imgui_frame &frames, const drawing &draw)
{
    tap(frames, draw, ImGuiKey_Enter);
}

}

#endif
