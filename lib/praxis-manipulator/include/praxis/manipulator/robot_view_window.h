#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_VIEW_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_VIEW_WINDOW_H

#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <span>
#include <string>
#include <vector>
#include <cstdint>
#include <optional>
#include <string_view>

namespace praxis::manipulator {

// Which drawings of the arm's model are shown: its meshes, the chain through its joint origins,
// both at once, or neither. Contiguous from zero, so a combo box indexes the label table with the
// enumerator's own value. The screw axes are a drawing under a control of their own and are not
// what this chooses: every entry leaves them wherever that control left them.
enum class model_render : std::uint8_t
{
    meshes,
    chain,
    meshes_and_chain,
    none
};

std::span<const char *const> model_render_labels();

// Which of the two drawings a rendering names. The screw axes are neither of them.
bool model_render_draws_meshes(model_render which);
bool model_render_draws_chain(model_render which);

// The drawings of one arm, how far a drawn screw axis reaches and how large the markers it carries
// are drawn, each reachable only where the composition asked for a control over it.
class robot_view_window : public scene::imgui_window, public config::configurable
{
public:
    // Which controls a composition offers. A control this does not ask for is not drawn, so the
    // feature behind it stays wherever the composition set it and nothing in the running
    // application can move it.
    struct controls
    {
        controls()
                : model(true)
                , reach(false)
                , decoration(true)
                , flange_marker(false)
                , tool_frame_marker(false)
                , marker_scale(false)
        {
        }

        bool model;
        bool reach;
        bool decoration;
        bool flange_marker;
        bool tool_frame_marker;
        bool marker_scale;
    };

    // What the window opens the arm at. An absent reach leaves the decoration at the reach the
    // stencil opened at, which is proportioned to the rendered arm's own size; a value names how far
    // a drawn axis runs either way of its anchor, in metres.
    struct settings
    {
        model_render model;
        bool decoration;
        std::optional<double> axis_reach;
        bool flange_marker;
        // Absent unless a document asks: a tool offset of the identity stands the marker at the tool
        // frame on top of the one at the flange, so neither reads as the frame it marks.
        bool tool_frame_marker;
        // A multiple of the size a marker was built at, so one document serves a marker built for a
        // small machine and one built for a large one.
        double marker_scale;

        explicit settings(model_render chosen_model = model_render::meshes, bool chosen_decoration = true, std::optional<double> chosen_reach = std::nullopt, bool chosen_marker = true,
                          bool chosen_tool_marker = false, double chosen_marker_scale = 1.0);
    };

    robot_view_window(std::string name, loadable_robot_stencil &target);
    robot_view_window(std::string name, loadable_robot_stencil &target, const controls &offered, const settings &state, std::string at = std::string());

    settings state() const;

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
    float m_reach;
    float m_marker_scale;
    bool m_marker;
    bool m_tool_marker;
    bool m_decoration;
    model_render m_model;

    // Whether the reach the window carries is one somebody named, rather than the size-proportional
    // one the stencil opened at. An unnamed reach is written out as absence, so a document keeps its
    // proportion to whichever arm reads it.
    bool m_reach_named;
    controls m_controls;
    std::string m_settings_at;
    loadable_robot_stencil &m_stencil;

    // Both drawings are written on every change, so no combination can be reached by one of them
    // being left where an earlier entry put it.
    void show_model();

    void render_model();
    void render_reach();
    void render_decoration();
    void render_flange_marker();
    void render_tool_frame_marker();
    void render_marker_scale();
};

}

#endif
