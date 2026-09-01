#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_PANEL_KEYS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_PANEL_KEYS_H

#include "imgui_frame.h"

#include <imgui.h>
#include <imgui_internal.h>

#include <cstddef>
#include <functional>

namespace praxis::fixture {

using drawing = std::function<void()>;

inline void tap(tests::imgui_frame &frames, const drawing &draw, ImGuiKey key)
{
    ImGui::GetIO().AddKeyEvent(key, true);
    frames.draw_frame(draw);
    ImGui::GetIO().AddKeyEvent(key, false);
    frames.draw_frame(draw);
}

// The first key a focused panel is given only raises the keyboard cursor, so the absolute step is
// taken twice: the second lands where the first would have whether or not the first was spent.
inline void reach_top(tests::imgui_frame &frames, const drawing &draw)
{
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    frames.draw(
            [&draw]
            {
                ImGui::SetNextWindowFocus();
                draw();
            });
    tap(frames, draw, ImGuiKey_Home);
    tap(frames, draw, ImGuiKey_Home);
}

inline ImGuiID standing_on()
{
    return ImGui::GetCurrentContext()->NavId;
}

// Every item the panel offers the keyboard, walked from its first downwards. The walk ends where a
// step down leaves the cursor where it was, so the panel is what says how long the walk is.
inline std::size_t navigable_items(tests::imgui_frame &frames, const drawing &draw)
{
    reach_top(frames, draw);

    std::size_t counted = 0;
    for(ImGuiID standing = 0; standing != standing_on(); ++counted)
    {
        standing = standing_on();
        tap(frames, draw, ImGuiKey_DownArrow);
    }

    return counted;
}

inline bool popup_open()
{
    return ImGui::GetCurrentContext()->OpenPopupStack.Size > 0;
}

// Presses whatever the cursor stands on and steps past it, once for every item the panel drew: a
// checkbox flips, a button is pressed, a slider takes a step. The second press is spent only on a
// control that opened a list, where it takes an entry and closes the list again; spending it on
// every control would return a checkbox to where it started and drive nothing.
inline void drive_every_control(tests::imgui_frame &frames, const drawing &draw)
{
    const std::size_t offered = navigable_items(frames, draw);

    reach_top(frames, draw);
    for(std::size_t step = 0; step < offered; ++step)
    {
        tap(frames, draw, ImGuiKey_Space);
        if(popup_open())
            tap(frames, draw, ImGuiKey_Space);
        tap(frames, draw, ImGuiKey_RightArrow);
        tap(frames, draw, ImGuiKey_DownArrow);
    }
}

// Puts the keyboard cursor that many steps below the panel's first item.
inline void stand_below_top(tests::imgui_frame &frames, const drawing &draw, std::size_t steps)
{
    reach_top(frames, draw);
    for(std::size_t step = 0; step < steps; ++step)
        tap(frames, draw, ImGuiKey_DownArrow);
}

// The step buttons of a value box sit to the right of the box on the same line, which a walk down a
// panel does not reach: nothing below the box is one of them.
inline void step_up_from(tests::imgui_frame &frames, const drawing &draw, std::size_t below_top)
{
    stand_below_top(frames, draw, below_top);
    tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_Space);
}

// A slider takes its steps from the keyboard only once it has been activated; the steps a control
// standing beside it would take move the cursor instead.
inline void tweak_from(tests::imgui_frame &frames, const drawing &draw, std::size_t below_top, std::size_t taken)
{
    stand_below_top(frames, draw, below_top);
    tap(frames, draw, ImGuiKey_Space);
    for(std::size_t step = 0; step < taken; ++step)
        tap(frames, draw, ImGuiKey_RightArrow);
}

// Types into a box standing that many places along the row the walk reached, replacing what it
// carries: activating a box selects its whole contents, so the characters delivered stand in for
// them rather than joining them, and the edit reaches the value only once it is committed. A row of
// components is entered at its leftmost and the rest of it is stepped along.
inline void type_into_component(tests::imgui_frame &frames, const drawing &draw, std::size_t below_top, std::size_t along, const char *typed)
{
    stand_below_top(frames, draw, below_top);
    for(std::size_t step = 0; step < along; ++step)
        tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_Space);
    for(const char *at = typed; *at != 0; ++at)
        ImGui::GetIO().AddInputCharacter(static_cast<unsigned int>(*at));
    frames.draw_frame(draw);
    tap(frames, draw, ImGuiKey_Enter);
}

inline void type_into(tests::imgui_frame &frames, const drawing &draw, std::size_t below_top, const char *typed)
{
    type_into_component(frames, draw, below_top, 0, typed);
}

// A slider takes typed characters only where it was activated for input rather than for stepping,
// which the library reserves for the Enter key: the key that opens a value box for typing puts a
// slider into its stepping mode instead.
inline void type_into_slider(tests::imgui_frame &frames, const drawing &draw, std::size_t below_top, const char *typed)
{
    stand_below_top(frames, draw, below_top);
    tap(frames, draw, ImGuiKey_Enter);
    for(const char *at = typed; *at != 0; ++at)
        ImGui::GetIO().AddInputCharacter(static_cast<unsigned int>(*at));
    frames.draw_frame(draw);
    tap(frames, draw, ImGuiKey_Enter);
}

// Whether the control the cursor stands on opened a list when it was activated, and, where it did,
// the entry below the one it opened on taken. A control that opened none is left as it was found:
// the activation is abandoned with the frame rather than committed.
inline bool entry_taken_at(tests::imgui_frame &frames, const drawing &draw, std::size_t below_top)
{
    stand_below_top(frames, draw, below_top);
    tap(frames, draw, ImGuiKey_Space);
    if(!popup_open())
        return false;

    tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_Space);

    return true;
}

// Drives whatever control stands that many steps below the panel's first item, whichever kind it
// is, stopping at the first instrument that moves it: a list has its next entry taken, a value box
// has the step button beside it pressed, a slider takes a keyboard step once activated. Which
// instrument fits comes from the panel rather than from a count of where it puts each control, and
// each attempt is made in a frame of its own, so one that fitted nothing commits nothing.
inline void drive_control_at(const drawing &draw, std::size_t below_top, const std::function<bool()> &unmoved)
{
    {
        tests::imgui_frame frames;
        frames.assert_on_frame_faults(true);
        if(entry_taken_at(frames, draw, below_top))
            return;
    }

    {
        tests::imgui_frame frames;
        frames.assert_on_frame_faults(true);
        step_up_from(frames, draw, below_top);
    }

    if(!unmoved())
        return;

    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);
    tweak_from(frames, draw, below_top, 1);
}

// Opens the combo the cursor stands on and takes the entry below the one it opens on.
inline void take_next_entry(tests::imgui_frame &frames, const drawing &draw)
{
    tap(frames, draw, ImGuiKey_Space);
    tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_Space);
}

}

#endif
