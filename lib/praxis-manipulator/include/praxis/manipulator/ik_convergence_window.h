#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_IK_CONVERGENCE_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_IK_CONVERGENCE_WINDOW_H

#include "praxis/manipulator/ik_iterate_window.h"

#include "praxis/scene/plot_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <string>
#include <vector>
#include <string_view>

namespace praxis::manipulator {

// How far one start still was from the target at each of its steps, drawn on a logarithmic ordinate
// so an error falling by a constant factor per step draws as a straight line. The start is the one
// the table beside this is about rather than a second choice of its own. An error of exactly zero
// has no place on that scale and is left out of the curve; the steps around it are still drawn.
class ik_convergence_window : public scene::plot_window, public config::configurable
{
public:
    struct settings
    {
        bool angular;
        bool linear;

        settings(bool chosen_angular = true, bool chosen_linear = true);
    };

    ik_convergence_window(std::string name, const ik_iterate_window &followed);
    ik_convergence_window(std::string name, const ik_iterate_window &followed, const settings &state, std::string at = std::string());

    settings state() const;

    // What one frame draws: a message stands in place of the curves wherever there is nothing to
    // draw, which a solve that reached no step and a solve whose steps are all off both leave.
    scene::plot_reading reading() const;

    std::string_view settings_path() const override
    {
        return m_settings_at;
    }

    std::vector<config::edit> settings_edits(const config::document &) const override;

    // A window no key path was named for has nowhere to write, so it offers nothing.
    const config::configurable *as_configurable() const override
    {
        return m_settings_at.empty() ? nullptr : this;
    }

private:
    bool m_linear;
    bool m_angular;
    std::string m_settings_at;
    const ik_iterate_window &m_followed;

    void render_controls();
};

}

#endif
