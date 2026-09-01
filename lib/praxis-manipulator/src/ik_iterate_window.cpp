#include "praxis/manipulator/ik_iterate_window.h"
#include "praxis/manipulator/kinematics_configuration.h"

#include "praxis/extension/held_handle.h"

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>

namespace praxis::manipulator {

namespace {

constexpr std::array<control_mode, 2> mode_options{control_mode::preview, control_mode::simulation};

}

ik_iterate_window::settings::settings(std::size_t chosen, control_mode chosen_mode)
        : start(chosen)
        , mode(chosen_mode)
{
}

ik_iterate_window::ik_iterate_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm)
        : ik_iterate_window(std::move(name), std::move(seen), std::move(arm), settings{})
{
}

ik_iterate_window::ik_iterate_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const settings &state, std::string at)
        : imgui_window(std::move(name))
        , m_seen(seen)
        , m_iterate(0u)
        , m_selected(state.start)
        , m_settings_at(std::move(at))
        , m_arm(std::move(arm))
        , m_control_mode(state.mode, mode_options, control_mode_labels())
{
    static_cast<void>(held(seen.read(), "the iterate table window", "published arm state"));
}

ik_iterate_window::settings ik_iterate_window::state() const
{
    return settings{m_selected, m_control_mode.value()};
}

std::optional<std::size_t> ik_iterate_window::standing_at(const std::shared_ptr<const arm_snapshot> &published) const
{
    if(published == nullptr || published->iterations.empty())
        return std::nullopt;

    return std::min(m_selected, published->iterations.size() - 1u);
}

std::optional<std::size_t> ik_iterate_window::selected() const
{
    return standing_at(m_seen.read());
}

std::vector<iteration_state> ik_iterate_window::iterates() const
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();
    const std::optional<std::size_t> which              = standing_at(published);

    return which ? published->iterations[*which] : std::vector<iteration_state>();
}

std::vector<config::edit> ik_iterate_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_ik_iterates(state(), m_settings_at));
}

void ik_iterate_window::relist(std::size_t starts)
{
    if(m_starts.size() == starts)
        return;

    m_starts.clear();
    for(std::size_t start = 0; start < starts; ++start)
        m_starts.push_back("s" + std::to_string(start + 1u));
}

void ik_iterate_window::step_to(const joint_vector &standing)
{
    const bool previewing = m_control_mode == control_mode::preview;

    command(m_arm,
            [standing, previewing](robot_controller &control, scene_robot &)
            {
                if(previewing)
                {
                    control.preview_joint_configuration(standing);

                    return;
                }

                const std::array<joint_vector, 1> target{standing};
                control.joint_space_trajectory(target);
            });
}

}
