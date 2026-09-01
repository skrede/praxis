#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_JOG_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_JOG_WINDOW_H

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/edited_pose.h"
#include "praxis/manipulator/control_mode.h"
#include "praxis/manipulator/option_cycle.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/types.h"

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <string_view>

namespace praxis::manipulator {

// A motion along and about one screw axis, taken from the start pose the composer shares in. The
// axis is named the way the screw parameters are named: `q` a point on it, `w` its direction and `h`
// the pitch in metres per radian; the angle is in degrees and the point in metres.
class screw_jog_window : public scene::imgui_window, public config::configurable
{
public:
    struct settings
    {
        control_mode mode;

        settings(control_mode chosen = control_mode::simulation);
    };

    screw_jog_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited);
    screw_jog_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited,
                     const settings &state, std::string at = std::string());

    settings state() const;

    void render() override;

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
    float m_pitch;
    arm_reader m_seen;
    Eigen::Vector3f m_q;
    Eigen::Vector3f m_w;
    float m_theta_degrees;
    std::string m_settings_at;
    std::weak_ptr<owned_arm> m_arm;
    rigid_motion::frame_ops m_frame;
    std::shared_ptr<edited_pose> m_edited;
    option_cycle<control_mode, 2> m_control_mode;

    void clear_screw();
    bool render_screw_axis();
    bool render_screw_start_pose(const arm_snapshot &seen);
    void render_screw_jog(const arm_snapshot &seen);

    void preview_screw();
    void move_along_screw(const Eigen::Vector3d &axis);
};

}

#endif
