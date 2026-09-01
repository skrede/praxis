#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_TRAJECTORY_PREVIEW_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_TRAJECTORY_PREVIEW_WINDOW_H

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/plot_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <string_view>

namespace praxis::manipulator {

// A motion asked for before it is played: its tool path drawn beside the path the tool last
// traversed, its samples scrubbed with the arm standing where the scrub indexes, and the scaling
// that drives it plotted against the two it was chosen over. The three frames share one
// time axis, and none of them is drawn on a logarithmic ordinate, because the second derivative of
// a time scaling goes negative.
class trajectory_preview_window : public scene::plot_window, public config::configurable
{
public:
    struct settings
    {
        bool parameter;
        bool rate;
        bool rate_change;

        settings(bool chosen_parameter = true, bool chosen_rate = true, bool chosen_rate_change = true);
    };

    // `ask` is the composer's route to whatever this preset previews, so the same window serves one
    // that previews a run of rows and one that previews a single motion, and neither this window nor
    // the composer branches on which it is.
    trajectory_preview_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, loadable_robot_stencil &drawn, std::function<void()> ask);
    trajectory_preview_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, loadable_robot_stencil &drawn, std::function<void()> ask, const settings &state,
                              std::string at = std::string());

    settings state() const;

    // What the three frames draw: a message stands in place of them wherever there is nothing to
    // draw, which no standing preview and a motion timing itself both leave.
    scene::plot_reading reading() const;

    // The drawings are told here rather than from the controls, so a collapsed panel still draws
    // what the arm published.
    void render() override;

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
    int m_scrub;
    bool m_rate;
    bool m_parameter;
    arm_reader m_seen;
    bool m_rate_change;
    std::string m_settings_at;
    std::weak_ptr<owned_arm> m_arm;
    std::function<void()> m_ask_cb;
    loadable_robot_stencil &m_drawn;
    // What the stencil was last told, held to compare the published handles against by pointer
    // identity: both are replaced whole and never appended to, so an unchanged handle is an
    // unchanged run and rebuilding a polyline from it would cost a thousand poses a frame.
    std::shared_ptr<const preview_run> m_told_preview;
    std::shared_ptr<const std::vector<transform>> m_told_traversed;

    void render_controls();
    void render_scrub(const preview_run &shown);

    // The published preview's tool poses under the commanded name and the traversed run under the
    // traversed name, each told only where its handle has changed.
    void draw_paths();
};

}

#endif
