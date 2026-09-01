#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_IK_ITERATE_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_IK_ITERATE_WINDOW_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/control_mode.h"
#include "praxis/manipulator/option_cycle.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::manipulator {

// The steps one start of the last solve took, as a table to read and to stand the arm on. Which
// start that is stays this window's own choice and is never taken from the arm: several starts
// reach one posture, so where the arm stands names no start, and two starts are compared without
// moving the arm at all. Nothing here reaches a solver, so reading a start leaves the steps it
// recorded unchanged.
class ik_iterate_window : public scene::imgui_window, public config::configurable
{
public:
    struct settings
    {
        std::size_t start;
        control_mode mode;

        settings(std::size_t chosen = 0u, control_mode chosen_mode = control_mode::simulation);
    };

    ik_iterate_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm);
    ik_iterate_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const settings &state, std::string at = std::string());

    settings state() const;

    // Which start everything drawn here is about, clamped to what the publication carries and
    // nothing where it carries no start. A start that recorded no step is still a start.
    std::optional<std::size_t> selected() const;

    // The steps of that start, and none where there is no start to be about. A second reader drawing
    // the same start some other way takes them from here, so the two cannot disagree about which
    // start they are about.
    std::vector<iteration_state> iterates() const;

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
    std::size_t m_iterate;
    std::size_t m_selected;
    std::string m_settings_at;
    std::weak_ptr<owned_arm> m_arm;
    std::vector<std::string> m_starts;
    option_cycle<control_mode, 2> m_control_mode;

    void step_to(const joint_vector &standing);
    void relist(std::size_t starts);
    void render_iterates(const arm_snapshot &seen);
    void render_steps(const std::vector<iteration_state> &steps);
    void render_table(const std::vector<iteration_state> &steps);
    std::optional<std::size_t> standing_at(const std::shared_ptr<const arm_snapshot> &published) const;
};

}

#endif
