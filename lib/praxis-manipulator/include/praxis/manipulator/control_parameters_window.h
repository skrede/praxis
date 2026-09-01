#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_CONTROL_PARAMETERS_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_CONTROL_PARAMETERS_WINDOW_H

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/option_cycle.h"
#include "praxis/manipulator/motion_commands.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <functional>
#include <string_view>

namespace praxis::manipulator {

// The three choices and the labels a control shows them under, in the order the enumerators are
// written. A document carries the label rather than the position, so reordering either table
// retires whatever documents already name the labels it moved.
inline constexpr std::array<time_scaling_choice, 3> time_scaling_options{time_scaling_choice::cubic, time_scaling_choice::quintic, time_scaling_choice::trapezoidal};
inline constexpr std::array<const char *, 3> time_scaling_labels{"Cubic", "Quintic", "Trapezoidal"};

// The playback rate and the time scaling every motion the arm makes takes, with the per-joint bounds
// the rate produces. The width is ImGui's: its float widget binds a float, and the arm's own double
// quantity is converted at the call.
class control_parameters_window : public scene::imgui_window, public config::configurable
{
public:
    struct settings
    {
        float velocity;
        time_scaling_choice scaling;
        std::optional<path_parameter_bounds> trapezoid;

        explicit settings(float chosen_velocity = 0.3f, time_scaling_choice chosen_scaling = time_scaling_choice::quintic,
                          std::optional<path_parameter_bounds> chosen_trapezoid = std::nullopt);
    };

    // `changed` is told where the time scaling or the trapezoid bounds are chosen anew, which are
    // the parameters a motion is composed under. The playback rate is not one of them: it scales a
    // motion already composed and leaves what was composed from these standing.
    control_parameters_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, std::function<void()> changed = {});
    control_parameters_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const settings &state, std::string at = std::string(),
                              std::function<void()> changed = {});

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
    float m_velocity;
    float m_max_rate;
    bool m_overridden;
    arm_reader m_seen;
    float m_max_rate_change;
    std::string m_settings_at;
    std::weak_ptr<owned_arm> m_arm;
    std::function<void()> m_changed_cb;
    option_cycle<time_scaling_choice, 3> m_scaling;

    std::optional<path_parameter_bounds> held_bounds() const;

    void render_playback_rate();
    void render_time_scaling();
    void render_trapezoid_bounds(const arm_snapshot &seen);
    void render_joint_bounds(const arm_snapshot &seen);
};

}

#endif
