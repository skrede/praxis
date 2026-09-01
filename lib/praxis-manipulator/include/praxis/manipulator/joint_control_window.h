#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_JOINT_CONTROL_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_JOINT_CONTROL_WINDOW_H

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/control_mode.h"
#include "praxis/manipulator/option_cycle.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <string_view>

namespace praxis::manipulator {

// One field or one slider per joint, in degrees, with the commands the mode resolves to.
class joint_control_window : public scene::imgui_window, public config::configurable
{
    enum class control_widgets : std::uint8_t
    {
        sliders,
        fields
    };

public:
    struct settings
    {
        control_mode mode;

        settings(control_mode chosen = control_mode::simulation);
    };

    joint_control_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm);
    joint_control_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const settings &state, std::string at = std::string());

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
    Eigen::VectorXf m_joint_positions;
    std::vector<std::string> m_joint_labels;
    option_cycle<control_mode, 2> m_control_mode;
    option_cycle<control_widgets, 2> m_control_widgets;

    bool render_joint_inputs(const arm_snapshot &seen);
    void render_joint_space(const arm_snapshot &seen);
    void render_joint_commands(const joint_vector &commanded, bool edited);
    void seed_fields(const arm_snapshot &seen);
    void preview_configuration(const joint_vector &commanded);
    void move_to_configuration(const joint_vector &commanded);
};

}

#endif
