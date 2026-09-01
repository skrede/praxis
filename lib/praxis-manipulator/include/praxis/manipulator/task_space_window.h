#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_TASK_SPACE_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_TASK_SPACE_WINDOW_H

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/edited_pose.h"
#include "praxis/manipulator/control_mode.h"
#include "praxis/manipulator/option_cycle.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/rigid_motion/frame.h"

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <string_view>

namespace praxis::manipulator {

// One target pose, reached either as a straight line or point to point. The pose is shared with the
// other windows the same composition builds against it: each of them holds a share of it, so it
// lives as long as the last of them and no ordering among them is required.
class task_space_window : public scene::imgui_window, public config::configurable
{
public:
    enum class motion_shape : std::uint8_t
    {
        ptp,
        lin
    };

    struct settings
    {
        motion_shape shape;
        control_mode mode;

        settings(motion_shape chosen_shape = motion_shape::ptp, control_mode chosen_mode = control_mode::simulation);
    };

    task_space_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited);
    task_space_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited,
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
    arm_reader m_seen;
    std::string m_settings_at;
    std::weak_ptr<owned_arm> m_arm;
    rigid_motion::frame_ops m_frame;
    std::shared_ptr<edited_pose> m_edited;
    option_cycle<control_mode, 2> m_control_mode;
    option_cycle<motion_shape, 2> m_motion_shape;

    void render_task_space(const arm_snapshot &seen);
    void render_task_space_preview(const arm_snapshot &seen);
    void render_task_space_lin_p2p(const arm_snapshot &seen);
};

}

#endif
