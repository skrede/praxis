#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_PANEL_LABELS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_PANEL_LABELS_H

#include "panel_keys.h"
#include "imgui_frame.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <string>
#include <cstddef>

namespace praxis::fixture {

// The identifier a control carries: the interface library hashes a label under the identifier stack
// the window stands on, and a window drawn between Begin and End with nothing pushed stands on its
// own identifier alone.
inline ImGuiID control_id(const char *window, const char *label)
{
    ImGuiWindow *const held = ImGui::FindWindowByName(window);
    REQUIRE(held != nullptr);

    return held->GetID(label);
}

// A label drawn inside PushID(scope) hashes under the identifier that push seeded the stack with,
// which is the one the window forms from the same number.
inline ImGuiID control_id(const char *window, std::size_t scope, const char *label)
{
    ImGuiWindow *const held = ImGui::FindWindowByName(window);
    REQUIRE(held != nullptr);

    return ImHashStr(label, 0, held->GetID(static_cast<int>(scope)));
}

// Walks down from wherever the cursor stands until it stands on the control asked for. The walk ends
// where a step down leaves the cursor where it was, so a label the panel does not draw fails the
// case by name rather than spinning.
inline void walk_onto(tests::imgui_frame &frames, const drawing &draw, ImGuiID wanted, const std::string &named)
{
    for(ImGuiID standing = 0; standing != standing_on();)
    {
        if(standing_on() == wanted)
            return;

        standing = standing_on();
        tap(frames, draw, ImGuiKey_DownArrow);
    }

    FAIL("the keyboard walk reaches no " + named);
}

inline void stand_on(tests::imgui_frame &frames, const drawing &draw, const char *window, const char *label)
{
    reach_top(frames, draw);
    walk_onto(frames, draw, control_id(window, label), "'" + std::string(label) + "' in '" + std::string(window) + "'");
}

inline void stand_on(tests::imgui_frame &frames, const drawing &draw, const char *window, std::size_t scope, const char *label)
{
    reach_top(frames, draw);
    walk_onto(frames, draw, control_id(window, scope, label), "'" + std::string(label) + "' in panel " + std::to_string(scope) + " of '" + std::string(window) + "'");
}

inline void press_on(tests::imgui_frame &frames, const drawing &draw, const char *window, const char *label)
{
    stand_on(frames, draw, window, label);
    tap(frames, draw, ImGuiKey_Space);
}

inline void press_on(tests::imgui_frame &frames, const drawing &draw, const char *window, std::size_t scope, const char *label)
{
    stand_on(frames, draw, window, scope, label);
    tap(frames, draw, ImGuiKey_Space);
}

inline void take_entry_on(tests::imgui_frame &frames, const drawing &draw, const char *window, const char *label)
{
    stand_on(frames, draw, window, label);
    take_next_entry(frames, draw);
}

inline void take_entry_on(tests::imgui_frame &frames, const drawing &draw, const char *window, std::size_t scope, const char *label)
{
    stand_on(frames, draw, window, scope, label);
    take_next_entry(frames, draw);
}

// Opens the list the control draws and takes the entry at that place in it, counted from its first:
// the list opens its keyboard cursor on its first entry rather than on the entry it is showing.
inline void enter_at(tests::imgui_frame &frames, const drawing &draw, std::size_t entry)
{
    tap(frames, draw, ImGuiKey_Space);
    for(std::size_t step = 0; step < entry; ++step)
        tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_Space);
}

inline void take_entry_on(tests::imgui_frame &frames, const drawing &draw, const char *window, const char *label, std::size_t entry)
{
    stand_on(frames, draw, window, label);
    enter_at(frames, draw, entry);
}

inline void take_entry_on(tests::imgui_frame &frames, const drawing &draw, const char *window, std::size_t scope, const char *label, std::size_t entry)
{
    stand_on(frames, draw, window, scope, label);
    enter_at(frames, draw, entry);
}

// A roster row carries the name of the thing it stands for rather than a label of its own, so it is
// the one control reached by where it stands. A roster draws its rows first and one per item, which
// puts a row at the place its own index names.
inline std::size_t row_step(std::size_t index)
{
    return index;
}

}

#endif
