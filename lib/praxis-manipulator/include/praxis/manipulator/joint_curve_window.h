#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_JOINT_CURVE_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_JOINT_CURVE_WINDOW_H

#include "praxis/manipulator/arm_snapshot.h"

#include "praxis/scene/plot_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <string>
#include <vector>
#include <cstddef>
#include <string_view>

namespace praxis::manipulator {

// Every joint of a previewed motion drawn against time: its position, its rate and the rate's own
// rate, in three frames over one shared abscissa. The three frames share a time axis and none of
// them is drawn on a logarithmic ordinate, because a joint's rate and its second derivative both go
// negative. This is a reading of the publication and nothing else -- it commands no motion and
// holds no share of the arm.
class joint_curve_window : public scene::plot_window, public config::configurable
{
public:
    struct settings
    {
        // The joints left out of all three frames, counted from zero. An empty list draws every
        // joint the publication carries.
        std::vector<std::size_t> hidden;

        settings(std::vector<std::size_t> left_out = std::vector<std::size_t>());
    };

    joint_curve_window(std::string name, arm_reader seen);
    joint_curve_window(std::string name, arm_reader seen, const settings &state, std::string at = std::string());

    settings state() const;

    // What the three frames draw: a message stands in place of them wherever there is nothing to
    // draw, which no standing preview and every joint switched off both leave.
    scene::plot_reading reading() const;

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
    arm_reader m_seen;
    std::string m_settings_at;
    std::vector<std::size_t> m_hidden;

    void render_controls();
};

}

#endif
