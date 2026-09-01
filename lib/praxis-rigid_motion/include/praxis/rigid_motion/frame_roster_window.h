#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_FRAME_ROSTER_WINDOW_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_FRAME_ROSTER_WINDOW_H

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include <array>
#include <string>
#include <cstddef>
#include <functional>
#include <string_view>

namespace praxis::rigid_motion {

// Creates, names and removes the frames of the stencil it drives, and shows what it refused beside
// the control that produced the refusal.
//
// A name is fixed at creation and cannot be edited afterwards: an arrangement document keys its
// placements by name, so a rename would orphan what was already written under the old one, and two
// frames sharing a name would leave that document ambiguous about which of them it places.
//
// The frame set is read from the stencil at every draw rather than held here, so a frame added or
// removed anywhere else is drawn without this window being told.
class frame_roster_window : public scene::imgui_window
{
public:
    // Where the one selection a composition carries is read and written, in the stencil's own index
    // space. This window holds none of its own, so the row it stands on and the frame every window
    // composed beside it is about are the same frame.
    struct selection_route
    {
        std::function<std::size_t()> read;
        std::function<void(std::size_t)> write;
    };

    // The axes and the body a created frame is built from, the noun a name this window generates is
    // built from, and where the selection it acts on lives, so the composition decides what a new
    // frame looks like, what it is called, and which frame is meant.
    frame_roster_window(std::string name, frame_stencil &target, axes_settings arriving, object_body carried, std::string stem, selection_route standing);

    // An empty name is one this window generates; the created frame becomes the selection. A name
    // any current frame carries is refused as unsupported input, on the same terms whether it was
    // typed or generated, and the fixed frame's name is one of those no frame may be created under.
    expected<std::size_t, refusal> create(std::string_view named);

    // Answers what the stencil answered: a frame another frame's placement is expressed in, and an
    // index past the end, are the stencil's refusals and are forwarded rather than restated.
    expected<void, refusal> remove_selected();

    void render() override;

private:
    // The interface library writes a typed name into the buffer it is handed, so that buffer is
    // fixed and a name longer than this is cut off at the field rather than in the frame set.
    static constexpr std::size_t name_limit = 48;

    std::size_t m_generated;
    axes_settings m_arriving;
    object_body m_carried;
    std::string m_stem;
    frame_stencil &m_stencil;
    selection_route m_standing;
    std::string m_refused_create;
    std::string m_refused_remove;
    std::array<char, name_limit> m_typed;

    std::string generated_name();
    bool carried_already(std::string_view named) const;

    void render_roster();
    void render_creation();
    void render_removal();
};

}

#endif
