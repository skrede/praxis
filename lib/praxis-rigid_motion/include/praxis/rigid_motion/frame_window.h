#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_FRAME_WINDOW_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_FRAME_WINDOW_H

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/axis_order.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <functional>
#include <string_view>

namespace praxis::rigid_motion {

// The panel list follows the object set when that set changes size: the panels drawn are then read
// back from the stencil, and a panel naming an object that is gone is gone with it. While the set
// keeps its size a panel is the source of truth for the values a person is editing, and nothing
// reads them back out from under them. Size is the whole of the signal, so an object added and one
// removed between two draws leave the panels as they were; every control that reaches the object set
// is drawn from this same path, so no two of them run between one draw and the next.
class frame_window : public scene::imgui_window, public config::configurable
{
public:
    // One entry per object of the stencil this drives, in that stencil's own order. An absent parent
    // names the frame the objects hang under.
    struct placement
    {
        axis_order order = axis_order::zyx;
        Eigen::Vector3f position{0.f, 0.f, 0.f};
        Eigen::Vector3f euler_degrees{0.f, 0.f, 0.f};
        std::optional<std::size_t> parent;
    };

    struct settings
    {
        std::vector<placement> objects;
    };

    // Which controls a composition offers. A control this does not ask for is not drawn, so the
    // feature behind it stays wherever the composition set it and nothing in the running
    // application can move it. The first panelled object is the first one the window offers a panel
    // for: an object before it is drawn by the stencil, is choosable as a parent, and has no
    // control here at all.
    struct controls
    {
        controls()
                : parent(true)
                , visibility(false)
                , first_panelled_object(0)
        {
        }

        bool parent;
        bool visibility;
        std::size_t first_panelled_object;
    };

    // A composition that names a selection source has this window draw the panel that source names
    // and no other; one that names none has it draw every panel it offers.
    frame_window(std::string name, frame_stencil &target, const frame_ops &injected, const controls &offered = controls(), std::function<std::size_t()> selecting = nullptr);
    frame_window(std::string name, frame_stencil &target, const frame_ops &injected, const settings &chosen, std::string at = std::string(), const controls &offered = controls(),
                 std::function<std::size_t()> selecting = nullptr);

    settings state() const;

    // The object whose panel a window drawing one panel draws, in the stencil's own index space, so
    // a readout beside this window reads the same object the panel edits.
    std::size_t selected_object() const
    {
        return m_selected;
    }

    void render() override;

    void initialize() override;

    std::string_view settings_path() const override
    {
        return m_settings_at;
    }

    std::vector<config::edit> settings_edits(const config::document &carried) const override;

    // A window no key path was named for has nowhere to write, so it offers nothing.
    const config::configurable *as_configurable() const override
    {
        return m_settings_at.empty() ? nullptr : this;
    }

private:
    frame_ops m_frame;
    settings m_opened;
    controls m_controls;
    std::size_t m_selected;
    frame_stencil &m_stencil;
    std::string m_refused;
    std::string m_settings_at;
    std::vector<placement> m_objects;
    std::vector<std::string> m_entries;
    std::function<std::size_t()> m_selecting;

    void resynchronize();
    placement panel_of(std::size_t index) const;

    void render_refusal();
    void render_told_object();
    void render_panel(std::size_t index);
    void render_object(std::size_t index);
    void render_parent(std::size_t index);
    void render_visibility(std::size_t index);

    void assign_pose(std::size_t index);
    void apply_parent(std::size_t index, std::optional<std::size_t> chosen);
    void refuse_parent(std::size_t index, std::optional<std::size_t> chosen);
};

}

#endif
