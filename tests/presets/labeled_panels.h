#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_LABELED_PANELS_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_LABELED_PANELS_H

#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/scene/imgui_window.h"

#include <cstddef>

namespace praxis::fixture {

inline void press_on(scene::imgui_window &panel, const char *label)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing over = [&panel] { panel.render(); };
    press_on(frames, over, panel.display_name().c_str(), label);
}

// A control an object's own panel draws is drawn inside the identifier scope that panel pushes, so
// it is named by the object it belongs to as well as by its label.
inline void press_on(scene::imgui_window &panel, std::size_t scope, const char *label)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing over = [&panel] { panel.render(); };
    press_on(frames, over, panel.display_name().c_str(), scope, label);
}

// Opens the list the control draws and takes the entry at that place in it, counted from its first.
inline void take_entry_on(scene::imgui_window &panel, const char *label, std::size_t entry)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing over = [&panel] { panel.render(); };
    take_entry_on(frames, over, panel.display_name().c_str(), label, entry);
}

inline void take_entry_on(scene::imgui_window &panel, std::size_t scope, const char *label, std::size_t entry)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing over = [&panel] { panel.render(); };
    take_entry_on(frames, over, panel.display_name().c_str(), scope, label, entry);
}

}

#endif
