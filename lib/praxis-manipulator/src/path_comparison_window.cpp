#include "path_comparison_paths.h"

#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/trajectory_configuration.h"

#include "praxis/extension/held_handle.h"

#include "praxis/rigid_motion/angles.h"

#include <imgui.h>

#include <spdlog/spdlog.h>

#include <Eigen/Core>

#include <span>
#include <string>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>
#include <string_view>

namespace praxis::manipulator {

namespace {

constexpr const char *unpublished_arm = "The arm has published nothing yet.";
constexpr const char *jointless_arm   = "The arm carries no joint, so there is no pair of ends to compare a path over.";

joint_vector radians_of(const Eigen::VectorXf &degrees)
{
    return degrees.cast<double>() * radians_per_degree;
}

}

path_comparison_window::settings::settings(joint_vector opening_first, joint_vector opening_second, bool drawing_joint_space, bool drawing_decoupled, bool drawing_screw,
                                           compared_path chosen)
        : first(std::move(opening_first))
        , second(std::move(opening_second))
        , joint_space(drawing_joint_space)
        , decoupled(drawing_decoupled)
        , screw(drawing_screw)
        , played(chosen)
{
}

path_comparison_window::path_comparison_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, loadable_robot_stencil &drawn, forward_kinematics_ops forward,
                                               screw_chain chain, trajectory::path_ops shapes)
        : path_comparison_window(std::move(name), std::move(seen), std::move(arm), drawn, forward, std::move(chain), shapes, settings{})
{
}

path_comparison_window::path_comparison_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, loadable_robot_stencil &drawn, forward_kinematics_ops forward,
                                               screw_chain chain, trajectory::path_ops shapes, const settings &state, std::string at)
        : imgui_window(std::move(name))
        , m_screw(state.screw)
        , m_decoupled(state.decoupled)
        , m_joint_space(state.joint_space)
        , m_seen(seen)
        , m_chain(std::move(chain))
        , m_settings_at(std::move(at))
        , m_fk(forward)
        , m_shapes(shapes)
        , m_played(every_compared_path(state.played))
        , m_arm(std::move(arm))
        , m_drawn(drawn)
        , m_told(false)
        , m_told_traversed()
{
    const std::shared_ptr<const arm_snapshot> share = seen.read();
    const std::size_t joints                        = static_cast<std::size_t>(held(share, "the path comparison window", "published arm state").joints.size());

    accept(state.first.size() == 0 ? opening_first(joints) : state.first, "first", joints, m_first);
    accept(state.second.size() == 0 ? opening_second(joints) : state.second, "second", joints, m_second);
}

void path_comparison_window::accept(const joint_vector &end, std::string_view named, std::size_t joints, Eigen::VectorXf &into)
{
    if(end.size() == static_cast<Eigen::Index>(joints))
    {
        into = (end * degrees_per_radian).cast<float>();
        return;
    }

    spdlog::error("praxis: 'manipulator.path_comparison_window' was given a {} end of {} joint values for an arm of {} joints, so it was not taken and the end beside it still "
                  "stands",
                  named, end.size(), joints);

    into = Eigen::VectorXf::Zero(static_cast<Eigen::Index>(joints));
}

path_comparison_window::settings path_comparison_window::state() const
{
    return settings{radians_of(m_first), radians_of(m_second), m_joint_space, m_decoupled, m_screw, m_played.value()};
}

std::vector<config::edit> path_comparison_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_path_comparison(state(), m_settings_at));
}

void path_comparison_window::render()
{
    draw_paths();
    draw_traversed();

    const std::shared_ptr<const arm_snapshot> published = m_seen.read();

    ImGui::Begin(display_name().c_str());
    if(published == nullptr)
        ImGui::TextUnformatted(unpublished_arm);
    else if(m_first.size() == 0)
        ImGui::TextUnformatted(jointless_arm);
    else
        render_controls();
    ImGui::End();
}

void path_comparison_window::render_controls()
{
    render_ends();
    render_switches();
    static_cast<void>(render_option_cycle("Played", m_played));
    if(ImGui::Button("Play"))
        play();
}

void path_comparison_window::render_ends()
{
    ImGui::InputScalarN("First", ImGuiDataType_Float, m_first.data(), static_cast<int>(m_first.size()), nullptr, nullptr, "%.1f");
    ImGui::InputScalarN("Second", ImGuiDataType_Float, m_second.data(), static_cast<int>(m_second.size()), nullptr, nullptr, "%.1f");
}

void path_comparison_window::render_switches()
{
    bool moved = ImGui::Checkbox(joint_space_path, &m_joint_space);
    moved      = ImGui::Checkbox(decoupled_path, &m_decoupled) || moved;
    moved      = ImGui::Checkbox(screw_path, &m_screw) || moved;
    if(moved)
        show_paths();
}

// The outer closure runs here, on the strand this panel is rendered on, and samples the shape from
// what this window holds; the inner one carries the samples by value and is the one thing that
// crosses onto the arm's strand. The arm is stood at the run's first sample directly, so what plays
// begins where the drawing does.
void path_comparison_window::play()
{
    const std::vector<joint_vector> through = configurations_along(m_shapes, radians_of(m_first), radians_of(m_second));
    if(through.size() < drawn_points)
        return;

    if(m_played == compared_path::joint_space)
    {
        command(m_arm,
                [through](robot_controller &control, scene_robot &driven)
                {
                    driven.set_joint_positions(through.front());
                    control.joint_space_trajectory(std::span<const joint_vector>(through));
                });

        return;
    }

    const std::vector<transform> poses = poses_along(m_played.value());
    if(poses.size() < drawn_points)
        return;

    command(m_arm,
            [first = through.front(), poses](robot_controller &control, scene_robot &driven)
            {
                driven.set_joint_positions(first);
                control.task_space_trajectory(std::span<const transform>(poses));
            });
}

}
