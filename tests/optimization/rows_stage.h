#ifndef HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_ROWS_STAGE_H
#define HPP_GUARD_PRAXIS_TESTS_OPTIMIZATION_ROWS_STAGE_H

#include "typed_stage.h"
#include "composed_curve.h"
#include "typed_function_panel.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/optimization/scenario_snapshot.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <memory>
#include <vector>
#include <cstddef>
#include <algorithm>

namespace praxis::fixture {

// The parameter rows of a typed curve drawn alone in a window, whose posted requests a drain runs.
struct rows_stage
{
    typed_stage stage;
    parameter_rows rows;

    explicit rows_stage(const char *text)
            : stage(typed_curve(text, 1.0))
            , rows(stage.taken.inputs->seen, stage.taken.inputs->scenario)
    {
    }

    drawing draw()
    {
        return [this]
        {
            ImGui::Begin("Rows");
            rows.render();
            ImGui::End();
        };
    }

    typed_parameter published(char name)
    {
        const std::shared_ptr<const scenario_snapshot> shown = stage.shown();
        const auto found                                     = std::ranges::find(shown->parameters, name, &typed_parameter::name);
        REQUIRE(found != shown->parameters.end());

        return *found;
    }

    double held(char name) const
    {
        const std::vector<typed_parameter> shown = rows.held();
        const auto found                         = std::ranges::find(shown, name, &typed_parameter::name);
        REQUIRE(found != shown.end());

        return found->value;
    }
};

// Activating a slider from the keyboard lets the arrows step it, and activating it again lets it go.
inline void slide(tests::imgui_frame &frames, const drawing &draw, char letter, std::size_t steps, bool letting_go = true)
{
    const char label[2] = {letter, '\0'};
    stand_on(frames, draw, "Rows", static_cast<std::size_t>(letter), label);
    tap(frames, draw, ImGuiKey_Space);
    for(std::size_t step = 0; step < steps; ++step)
        tap(frames, draw, ImGuiKey_RightArrow);
    if(letting_go)
        tap(frames, draw, ImGuiKey_Space);
}

}

#endif
