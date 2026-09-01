#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_TRAJECTORY_RECORDING_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_TRAJECTORY_RECORDING_WINDOW_H

#include "praxis/manipulator/arm_state.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <memory>
#include <string>
#include <vector>
#include <string_view>

namespace praxis::manipulator {

class trajectory_recording_window : public scene::imgui_window, public config::configurable
{
public:
    using settings = recording_parameters;

    trajectory_recording_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm);
    trajectory_recording_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const settings &state, std::string at = std::string());

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
    bool m_active;
    arm_reader m_seen;
    char m_directory_path[1024];
    std::string m_settings_at;
    std::weak_ptr<owned_arm> m_arm;
};

}

#endif
