#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_COMPOSED_PANELS_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_COMPOSED_PANELS_H

#include "panel_keys.h"
#include "imgui_frame.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/strand.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>

namespace praxis::fixture {

// The window context initializes a window as it takes it, which is where a window that pushes to a
// stencil on construction actually pushes.
inline scene::window_route opening_route()
{
    return [](const std::shared_ptr<scene::imgui_window> &panel) { panel->initialize(); };
}

inline scene::preset_site unwired(threepp::Scene &target)
{
    const scene::window_route nowhere = [](const std::shared_ptr<scene::imgui_window> &) {};

    return scene::preset_site{target, scheduler::strand{}, scheduler::strand{}, [] {}, nowhere, nowhere, {}};
}

// One headless scene a composition is built against, and the count of what hangs in it, which is how
// a scenario is held to leaving the scene as it found it.
struct stage
{
    stage()
            : scene(threepp::Scene::create())
    {
    }

    scene::preset_site site() const
    {
        return unwired(*scene);
    }

    std::size_t descendants() const
    {
        std::size_t counted = 0;
        scene->traverse([&counted](threepp::Object3D &) { ++counted; });

        return counted;
    }

    std::shared_ptr<threepp::Scene> scene;
};

inline std::vector<std::string> composed_windows(const std::shared_ptr<scene::preset> &composed)
{
    REQUIRE(composed != nullptr);

    std::vector<std::string> named;
    for(const std::shared_ptr<scene::imgui_window> &panel : composed->windows)
        named.push_back(panel->display_name());

    return named;
}

// The interface library identifies a panel by its title, so what one window opened in a frame is the
// set of titles the context holds once that frame has been drawn. The implicit debug panel a frame
// always carries is the library's own, and a child region is part of the panel that opened it.
inline std::vector<std::string> panels_opened_by(scene::imgui_window &panel)
{
    tests::imgui_frame frames;
    frames.draw([&panel] { panel.render(); });

    std::vector<std::string> named;
    for(const ImGuiWindow *held : ImGui::GetCurrentContext()->Windows)
        if(held->WasActive && (held->Flags & ImGuiWindowFlags_ChildWindow) == 0 && held->Name != std::string("Debug##Default"))
            named.push_back(held->Name);

    return named;
}

// Every vertex one drawing produced, so two panels are compared by what they put on screen rather
// than by how much of it there was.
inline std::size_t geometry_of(scene::imgui_window &panel)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);
    frames.draw([&panel] { panel.render(); });

    REQUIRE(frames.has_draw_data());
    REQUIRE(frames.vertices() > 0);

    return frames.signature();
}

inline void press_at(scene::imgui_window &panel, std::size_t steps)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing over = [&panel] { panel.render(); };
    stand_below_top(frames, over, steps);
    tap(frames, over, ImGuiKey_Space);
}

inline void step_at(scene::imgui_window &panel, std::size_t steps)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    step_up_from(frames, [&panel] { panel.render(); }, steps);
}

inline void tweak_at(scene::imgui_window &panel, std::size_t steps, std::size_t taken)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    tweak_from(frames, [&panel] { panel.render(); }, steps, taken);
}

inline void type_at(scene::imgui_window &panel, std::size_t steps, const char *typed)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    type_into(frames, [&panel] { panel.render(); }, steps, typed);
}

inline void type_component_at(scene::imgui_window &panel, std::size_t steps, std::size_t along, const char *typed)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    type_into_component(frames, [&panel] { panel.render(); }, steps, along, typed);
}

inline void type_into_slider_at(scene::imgui_window &panel, std::size_t steps, const char *typed)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    type_into_slider(frames, [&panel] { panel.render(); }, steps, typed);
}

// Opens the list the control that many steps below the panel's first item draws, and takes the entry
// at that place in it, counted from its first: the list opens its keyboard cursor on its first entry
// rather than on the entry it is showing, so the count is a position in the list.
inline void take_entry_at(scene::imgui_window &panel, std::size_t steps, std::size_t entry)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing over = [&panel] { panel.render(); };
    stand_below_top(frames, over, steps);
    tap(frames, over, ImGuiKey_Space);
    for(std::size_t step = 0; step < entry; ++step)
        tap(frames, over, ImGuiKey_DownArrow);
    tap(frames, over, ImGuiKey_Space);
}

// Every window a preset composes, held to the rule that a window is a panel: a window opening a
// second panel is what the arm's controls were before they were composed rather than switched.
inline void each_window_opens_one_panel(const std::shared_ptr<scene::preset> &composed)
{
    REQUIRE(composed != nullptr);

    for(const std::shared_ptr<scene::imgui_window> &panel : composed->windows)
    {
        INFO(panel->display_name());
        REQUIRE(panels_opened_by(*panel) == std::vector<std::string>{panel->display_name()});
    }
}

}

#endif
