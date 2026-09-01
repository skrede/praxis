#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_FRAME_SELECTOR_WINDOW_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_FRAME_SELECTOR_WINDOW_H

#include "praxis/rigid_motion/frame_stencil.h"

#include "praxis/scene/imgui_window.h"

#include <string>
#include <vector>
#include <cstddef>

namespace praxis::rigid_motion {

// Carries the choice of which frame the windows composed beside it are about, and the axis
// visibility of that one frame.
//
// What it offers is the objects of the stencil, read back at every draw rather than held here, so a
// frame added or removed anywhere else is offered without this window being told. The frame those
// objects hang under is not among them: it has no placement to edit and takes no index.
//
// A stencil carrying no objects offers nothing and leaves the choice standing at index zero.
class frame_selector_window : public scene::imgui_window
{
public:
    frame_selector_window(std::string name, frame_stencil &target);

    // The frame this window stands on, in the stencil's own index space, so a placement panel and a
    // matrix readout composed beside this window follow the frame this one names.
    std::size_t selected_object() const
    {
        return m_selected;
    }

    // Brought inside the object set the stencil carries rather than inside the entries drawn: a
    // caller writing immediately after a removal has not given this window a draw to refresh them.
    void select_object(std::size_t index);

    void render() override;

private:
    std::size_t m_selected;
    frame_stencil &m_stencil;
    std::vector<std::string> m_entries;

    void resynchronize();
    void render_axes_shown();
    void render_selected_object();
};

}

#endif
