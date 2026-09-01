#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/ik_branch_window.h"
#include "praxis/manipulator/kinematics_configuration.h"

#include "praxis/extension/held_handle.h"

#include "praxis/scene/widgets.h"

#include <imgui.h>

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <utility>

namespace praxis::manipulator {

namespace {

constexpr std::array<control_mode, 2> mode_options{control_mode::preview, control_mode::simulation};

constexpr const char *unpublished_arm = "The arm has published nothing yet.";
constexpr const char *nothing_found   = "The last solve found no configuration.";

bool same_configurations(const std::vector<joint_vector> &held, const std::vector<joint_vector> &taken)
{
    if(held.size() != taken.size())
        return false;

    for(std::size_t at = 0; at < held.size(); ++at)
        if(held[at].size() != taken[at].size() || !(held[at].array() == taken[at].array()).all())
            return false;

    return true;
}

}

ik_branch_window::settings::settings(control_mode chosen, bool chosen_figures)
        : mode(chosen)
        , figures(chosen_figures)
{
}

ik_branch_window::ik_branch_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited,
                                   loadable_robot_stencil &target, solve_route asked)
        : ik_branch_window(std::move(name), std::move(seen), std::move(arm), injected, std::move(edited), target, std::move(asked), settings{})
{
}

ik_branch_window::ik_branch_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited,
                                   loadable_robot_stencil &target, solve_route asked, const settings &state, std::string at)
        : imgui_window(std::move(name))
        , m_figures(state.figures)
        , m_seen(seen)
        , m_asked(std::move(asked))
        , m_selected(0u)
        , m_settings_at(std::move(at))
        , m_arm(std::move(arm))
        , m_frame(injected)
        , m_stencil(target)
        , m_edited(std::move(edited))
        , m_control_mode(state.mode, mode_options, control_mode_labels())
{
    static_cast<void>(held(seen.read(), "the branch list window", "published arm state"));
    static_cast<void>(held(m_edited, "the branch list window", "shared pose"));
}

ik_branch_window::settings ik_branch_window::state() const
{
    return settings{m_control_mode.value(), m_figures};
}

std::optional<std::size_t> ik_branch_window::selected() const
{
    return m_entries.empty() ? std::optional<std::size_t>() : std::optional<std::size_t>(m_selected);
}

std::vector<config::edit> ik_branch_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_ik_branch(state(), m_settings_at));
}

void ik_branch_window::initialize()
{
    m_stencil.set_solution_figures_shown(m_figures);
}

void ik_branch_window::render()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();

    ImGui::Begin(display_name().c_str());
    if(published == nullptr)
        ImGui::TextUnformatted(unpublished_arm);
    else
        render_branches(*published);
    ImGui::End();
}

void ik_branch_window::render_branches(const arm_snapshot &seen)
{
    relist(seen);
    tell_figures(seen);

    render_option_cycle("Control mode", m_control_mode);
    if(ImGui::Checkbox("Other solutions", &m_figures))
        m_stencil.set_solution_figures_shown(m_figures);

    if(m_entries.empty())
        ImGui::TextUnformatted(nothing_found);
    else if(scene::render_dropdown_selection("Solution", m_selected, m_entries))
        move_to(m_listed[m_selected]);

    if(ImGui::Button("Solve"))
        ask();
}

void ik_branch_window::relist(const arm_snapshot &seen)
{
    if(same_configurations(m_listed, seen.solutions))
        return;

    m_listed = seen.solutions;
    m_entries.clear();
    for(std::size_t which = 0; which < m_listed.size(); ++which)
        m_entries.push_back("b" + std::to_string(which + 1u));

    m_selected = solution_at(seen).value_or(0u);
}

void ik_branch_window::tell_figures(const arm_snapshot &seen)
{
    const std::optional<std::size_t> standing = solution_at(seen);

    std::vector<joint_vector> beside;
    for(std::size_t which = 0; which < seen.solutions.size(); ++which)
        if(standing != which)
            beside.push_back(seen.solutions[which]);

    if(same_configurations(m_beside, beside))
        return;

    m_beside = std::move(beside);
    static_cast<void>(m_stencil.set_solution_figures(m_beside));
}

void ik_branch_window::ask()
{
    if(!m_asked)
        return;

    solve_command asking = m_asked(pose_matrix(*m_edited, m_frame));
    if(!asking)
        return;

    command(m_arm, [asking = std::move(asking)](robot_controller &control, scene_robot &) { asking(control); });
}

void ik_branch_window::move_to(const joint_vector &commanded)
{
    const bool previewing = m_control_mode == control_mode::preview;

    command(m_arm,
            [commanded, previewing](robot_controller &control, scene_robot &)
            {
                if(previewing)
                {
                    control.preview_joint_configuration(commanded);

                    return;
                }

                const std::array<joint_vector, 1> target{commanded};
                control.joint_space_trajectory(target);
            });
}

}
