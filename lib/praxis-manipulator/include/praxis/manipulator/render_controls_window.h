#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_RENDER_CONTROLS_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_RENDER_CONTROLS_WINDOW_H

#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <array>
#include <string>
#include <vector>
#include <optional>
#include <string_view>

namespace praxis::manipulator {

// How large the drawings taken from one Jacobian stand: each block's manipulability ellipsoid, the
// cut a force ellipsoid is bounded at, and the arrows a column is drawn as. Every one of them turns
// a published number into a drawn length and none of them reaches what is computed.
class render_controls_window : public scene::imgui_window, public config::configurable
{
public:
    // Which controls a composition offers. A control this does not ask for is not drawn, so the
    // feature behind it stays wherever the composition set it and nothing in the running
    // application can move it.
    struct controls
    {
        bool ellipsoid_scale = true;
        bool force_cap       = true;
        bool column_scale    = true;
    };

    // What the window opens the drawings at. An absent value leaves it wherever the stencil opened
    // it, which is proportioned to the rendered arm; a length names it in drawn metres, the same
    // meaning `robot_view_window::settings::axis_reach` carries, and the cut names a multiple of the
    // block's own ellipsoid scale.
    struct settings
    {
        std::optional<double> angular_scale        = std::nullopt;
        std::optional<double> linear_scale         = std::nullopt;
        std::optional<double> angular_column_scale = std::nullopt;
        std::optional<double> linear_column_scale  = std::nullopt;
        std::optional<double> force_cap_ratio      = std::nullopt;
    };

    render_controls_window(std::string name, loadable_robot_stencil &drawn);
    render_controls_window(std::string name, loadable_robot_stencil &drawn, const controls &offered, const settings &state, std::string at = std::string());

    // Each length is present only where a control moved it or a document named it, so a save writes
    // what was chosen rather than what the stencil opened at.
    settings state() const;

    void render() override;

    // Every value reaches the stencil here whether or not a control was drawn for it, which is what
    // leaves a length nobody offered a control for standing where the composition put it.
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
    bool m_cap_named;
    std::array<bool, jacobian_block_count> m_scale_named;
    std::array<bool, jacobian_block_count> m_column_named;
    controls m_controls;
    std::string m_settings_at;
    loadable_robot_stencil &m_drawn;
    float m_force_cap_ratio;
    std::array<float, jacobian_block_count> m_scale;
    std::array<float, jacobian_block_count> m_column_scale;

    void render_cap();

    // The two ellipsoid scales and the two column scales are four fields of one kind, so one control
    // draws either pair and the pair it writes is what tells them apart.
    void render_lengths(bool ellipsoids);
};

}

#endif
