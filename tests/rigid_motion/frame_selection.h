#ifndef HPP_GUARD_PRAXIS_TESTS_RIGID_MOTION_FRAME_SELECTION_H
#define HPP_GUARD_PRAXIS_TESTS_RIGID_MOTION_FRAME_SELECTION_H

#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/frame_selector_window.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <string>
#include <vector>
#include <cstddef>

namespace praxis::fixture {

inline std::vector<rigid_motion::stencil_object> selectable_objects()
{
    rigid_motion::object_body bare;
    bare.shape = rigid_motion::body_shape::none;

    return {rigid_motion::stencil_object{"One", rigid_motion::axes_settings{}, bare}, rigid_motion::stencil_object{"Two", rigid_motion::axes_settings{}, bare},
            rigid_motion::stencil_object{"Three", rigid_motion::axes_settings{}, bare}};
}

// One stencil and one selector over it, standing where a composition would have left the two. The
// fixed frame carries a name of its own, so a list that offered it would be seen to.
struct selecting
{
    selecting()
            : scene()
            , body(scene, selectable_objects(), rigid_motion::baseline().frame, rigid_motion::fixed_frame{"Ground", rigid_motion::axes_settings{}})
            , selector("Selector", body)
    {
        REQUIRE(body.initialize().has_value());
    }

    selecting(const selecting &) = delete;

    threepp::Scene scene;
    rigid_motion::frame_stencil body;
    rigid_motion::frame_selector_window selector;
};

inline drawing over(rigid_motion::frame_selector_window &panel)
{
    return [&panel] { panel.render(); };
}

inline void draw_once(rigid_motion::frame_selector_window &panel)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);
    frames.draw(over(panel));

    REQUIRE(frames.has_draw_data());
}

inline void take_entry_on(rigid_motion::frame_selector_window &panel, const char *label, std::size_t entry)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing draw = over(panel);
    take_entry_on(frames, draw, panel.display_name().c_str(), label, entry);
}

inline void press_on(rigid_motion::frame_selector_window &panel, const char *label)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing draw = over(panel);
    press_on(frames, draw, panel.display_name().c_str(), label);
}

// Walks the panel from its first control onto each label in turn. The walk only goes downwards, so
// reaching them in the order given is what says the panel draws them in it.
inline void walk_onto_each(rigid_motion::frame_selector_window &panel, const std::vector<const char *> &labels)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing draw       = over(panel);
    const char *const window = panel.display_name().c_str();

    reach_top(frames, draw);
    for(const char *const label : labels)
        walk_onto(frames, draw, control_id(window, label), "'" + std::string(label) + "' below the control before it");
}

// The list a control offers is drawn inside the popup the interface library opens for it, so what
// the list carries is read from that popup rather than from the window the control sits in.
inline ImGuiWindow *opened_list()
{
    for(ImGuiWindow *const held : ImGui::GetCurrentContext()->Windows)
        if(held->WasActive && (held->Flags & ImGuiWindowFlags_Popup) != 0)
            return held;

    return nullptr;
}

// How many entries the list carries, and how many of them each of the asked-about names is drawn
// under. An entry is a selectable hashed under the popup, so a name is offered exactly where an
// entry the keyboard cursor can stand on carries that name's identifier.
struct offering
{
    std::size_t entries;
    std::vector<std::size_t> carried;
};

inline offering offered(rigid_motion::frame_selector_window &panel, const std::vector<std::string> &named)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing draw = over(panel);
    stand_on(frames, draw, panel.display_name().c_str(), "Object");
    tap(frames, draw, ImGuiKey_Space);

    ImGuiWindow *const list = opened_list();
    REQUIRE(list != nullptr);

    offering read{0, std::vector<std::size_t>(named.size(), 0)};
    for(ImGuiID standing = 0; standing != standing_on(); ++read.entries)
    {
        standing = standing_on();
        for(std::size_t asked = 0; asked < named.size(); ++asked)
            if(standing == list->GetID(named[asked].c_str()))
                ++read.carried[asked];
        tap(frames, draw, ImGuiKey_DownArrow);
    }

    return read;
}

}

#endif
