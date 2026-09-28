#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_TOOL_JOG_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_TOOL_JOG_WINDOW_H

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/edited_pose.h"
#include "praxis/manipulator/control_mode.h"
#include "praxis/manipulator/option_cycle.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/rigid_motion/frame.h"

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <string_view>

namespace praxis::manipulator {

// An offset and a rotation applied in the tool frame of a start pose. The start pose is the share the
// composer hands in; the offset and the rotation are the window's own and are in metres and degrees.
class tool_jog_window : public scene::imgui_window, public config::configurable
{
public:
    struct settings
    {
        control_mode mode;

        settings(control_mode chosen = control_mode::simulation);
    };

    tool_jog_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited);
    tool_jog_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited, const settings &state,
                    std::string at = std::string());

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
    arm_reader m_seen;
    std::string m_settings_at;
    std::weak_ptr<owned_arm> m_arm;
    Eigen::Vector3f m_jog_position;
    rigid_motion::frame_ops m_frame;
    Eigen::Vector3f m_jog_euler_degrees;
    std::shared_ptr<edited_pose> m_edited;
    option_cycle<control_mode, 2> m_control_mode;

    bool render_jog_start_pose(const arm_snapshot &seen);
    void render_tool_frame_jog(const arm_snapshot &seen);
};

}

#endif
