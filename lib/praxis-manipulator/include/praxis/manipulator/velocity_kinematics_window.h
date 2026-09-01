#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_VELOCITY_KINEMATICS_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_VELOCITY_KINEMATICS_WINDOW_H

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/option_cycle.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/labeled_value_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <string_view>

namespace praxis::manipulator {

// One Jacobian read as a matrix, as the two manipulability ellipsoids its blocks trace out, and as
// the arrows its columns stand for. The frame control governs all three together, so the numbers
// read here and the bodies drawn beside them can never stand for different matrices. Lynch & Park,
// Modern Robotics, sections 5.1 and 5.4.
class velocity_kinematics_window : public scene::labeled_value_window, public config::configurable
{
public:
    // Which controls a composition offers. A control this does not ask for is not drawn, so the
    // feature behind it stays wherever the composition set it and nothing in the running
    // application can move it.
    struct controls
    {
        bool frame   = true;
        bool reading = true;
        bool shown   = true;
    };

    // Which matrix the window reads, which of the two readings its blocks are taken under, and which
    // of the four drawings stand.
    struct settings
    {
        jacobian_frame frame   = jacobian_frame::space;
        ellipsoid_view reading = ellipsoid_view::velocity;
        bool angular_ellipsoid = true;
        bool linear_ellipsoid  = true;
        bool columns           = true;
        bool capped            = true;
    };

    velocity_kinematics_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, loadable_robot_stencil &drawn);
    velocity_kinematics_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, loadable_robot_stencil &drawn, const controls &offered, const settings &state,
                               std::string at = std::string());

    settings state() const;

    // Every value reaches the stencil here whether or not a control was drawn for it, which is what
    // leaves a feature nobody offered a control for standing where the composition put it.
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

    // The matrix the window is set to and the numbers its blocks give, as the panel draws them. Each
    // block's drawn extent is taken from the stencil, so the number read and the body drawn cannot
    // stand at different scales. A reading is also where a drawn length that is not a finite number
    // is named, once, so nothing else has to ask whether one is.
    scene::readout reading() const;

    // The name a drawn length that is not a finite number is refused under.
    static constexpr const char *unbounded_ellipsoid = "dk.manipulability_ellipsoid";

private:
    bool m_capped;
    bool m_columns;
    std::array<bool, jacobian_block_count> m_shown;

    // Whether a drawn length that is not a finite number has already been named. It is a reading's
    // own latch, so a standing runaway is reported once rather than once a frame.
    mutable bool m_refused;
    controls m_controls;
    std::string m_settings_at;
    arm_reader m_seen;
    std::weak_ptr<owned_arm> m_arm;
    loadable_robot_stencil &m_drawn;
    option_cycle<jacobian_frame, 2> m_frame;
    option_cycle<ellipsoid_view, 2> m_reading;

    void render_controls();
    void render_frame();
    void render_reading();
    void render_switches();

    // Named through the arm's own strand, which is the only one that can name a refusal.
    void report_unbounded() const;
};

}

#endif
