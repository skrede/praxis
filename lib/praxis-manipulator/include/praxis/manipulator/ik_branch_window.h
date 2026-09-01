#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_IK_BRANCH_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_IK_BRANCH_WINDOW_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/edited_pose.h"
#include "praxis/manipulator/control_mode.h"
#include "praxis/manipulator/option_cycle.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/rigid_motion/frame.h"

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <functional>
#include <string_view>

namespace praxis::manipulator {

// The answers one solve found, as a list to pick from rather than as a picture, and the control the
// solve is asked for through. A solve happens where the learner asks for one and at no other time:
// moving the target pose or a start changes nothing here until the ask is made.
class ik_branch_window : public scene::imgui_window, public config::configurable
{
public:
    // What one ask does to the arm, run on the arm's own strand.
    using solve_command = std::function<void(robot_controller &)>;

    // Which solve one ask makes, answered on the strand the ask was made on so a composition reads
    // its own controls where they are edited and hands back only what the arm's strand needs. A
    // route answering nothing asks for nothing. Which slot is reached is the composition's to say:
    // nothing here names a solver.
    using solve_route = std::function<solve_command(const transform &)>;

    struct settings
    {
        control_mode mode;
        bool figures;

        settings(control_mode chosen = control_mode::simulation, bool chosen_figures = true);
    };

    ik_branch_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited,
                     loadable_robot_stencil &target, solve_route asked);
    ik_branch_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited,
                     loadable_robot_stencil &target, solve_route asked, const settings &state, std::string at = std::string());

    settings state() const;

    // Which entry of the list the control stands on, and nothing where the list is empty.
    std::optional<std::size_t> selected() const;

    void render() override;

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

private:
    bool m_figures;
    arm_reader m_seen;
    solve_route m_asked;
    std::size_t m_selected;
    std::string m_settings_at;
    std::weak_ptr<owned_arm> m_arm;
    rigid_motion::frame_ops m_frame;
    loadable_robot_stencil &m_stencil;
    std::vector<joint_vector> m_beside;
    std::vector<joint_vector> m_listed;
    std::vector<std::string> m_entries;
    std::shared_ptr<edited_pose> m_edited;
    option_cycle<control_mode, 2> m_control_mode;

    void ask();
    void relist(const arm_snapshot &seen);
    void tell_figures(const arm_snapshot &seen);
    void move_to(const joint_vector &commanded);
    void render_branches(const arm_snapshot &seen);
};

}

#endif
