#include "praxis/manipulator/motion_drawings.h"
#include "praxis/manipulator/trajectory_configuration.h"
#include "praxis/manipulator/trajectory_preview_window.h"

#include "trajectory_preview_frames.h"

#include <imgui.h>

#include <threepp/math/Color.hpp>

#include <string>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>
#include <functional>

namespace praxis::manipulator {

namespace {

constexpr const char *unpublished_arm = "The arm has published nothing yet.";

// A sample whose pose the solver refused carries no position to draw through, and the samples
// around it are still drawn.
std::vector<transform> tool_poses_of(const preview_run &shown)
{
    std::vector<transform> through;
    through.reserve(shown.samples.size());
    for(const preview_sample &one : shown.samples)
        if(one.tool_pose)
            through.push_back(*one.tool_pose);

    return through;
}

}

trajectory_preview_window::settings::settings(bool chosen_parameter, bool chosen_rate, bool chosen_rate_change)
        : parameter(chosen_parameter)
        , rate(chosen_rate)
        , rate_change(chosen_rate_change)
{
}

trajectory_preview_window::trajectory_preview_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, loadable_robot_stencil &drawn, std::function<void()> ask)
        : trajectory_preview_window(std::move(name), std::move(seen), std::move(arm), drawn, std::move(ask), settings{})
{
}

trajectory_preview_window::trajectory_preview_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, loadable_robot_stencil &drawn, std::function<void()> ask,
                                                     const settings &state, std::string at)
        : plot_window(
                  std::move(name), [this] { render_controls(); }, [this] { return reading(); })
        , m_scrub(0)
        , m_rate(state.rate)
        , m_parameter(state.parameter)
        , m_seen(std::move(seen))
        , m_rate_change(state.rate_change)
        , m_settings_at(std::move(at))
        , m_arm(std::move(arm))
        , m_ask_cb(std::move(ask))
        , m_drawn(drawn)
        , m_told_preview()
        , m_told_traversed()
{
}

trajectory_preview_window::settings trajectory_preview_window::state() const
{
    return settings{m_parameter, m_rate, m_rate_change};
}

scene::plot_reading trajectory_preview_window::reading() const
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();
    const shown_frames chosen{m_parameter, m_rate, m_rate_change};

    return published == nullptr ? plotted_preview(nullptr, time_scaling_choice::quintic, chosen) : plotted_preview(published->preview.get(), published->time_scaling, chosen);
}

void trajectory_preview_window::render()
{
    draw_paths();
    plot_window::render();
}

std::vector<config::edit> trajectory_preview_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_trajectory_preview(state(), m_settings_at));
}

// The outer closure runs here, on the strand this panel is rendered on, and reads whatever it needs
// from this window; the inner one carries values only and is the one thing that crosses onto the
// arm's strand.
void trajectory_preview_window::render_controls()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();
    if(published == nullptr)
    {
        ImGui::TextUnformatted(unpublished_arm);
        return;
    }

    if(m_ask_cb && ImGui::Button("Preview"))
        m_ask_cb();

    const std::shared_ptr<const preview_run> shown = published->preview;
    if(shown != nullptr)
    {
        ImGui::SameLine();
        if(ImGui::Button("Play"))
            command(m_arm, [](robot_controller &control, scene_robot &) { control.play_preview(); });
        render_scrub(*shown);
    }

    ImGui::Checkbox(parameter_axis, &m_parameter);
    ImGui::SameLine();
    ImGui::Checkbox(rate_axis, &m_rate);
    ImGui::SameLine();
    ImGui::Checkbox(rate_change_axis, &m_rate_change);
}

// A queued run concatenates one leg per waypoint pair and every leg carries its own path parameter
// running zero to one, so that parameter is not monotone across the run and does not index it. The
// samples are, and the instant the indexed one stands at is stated beside the control in seconds,
// which is the abscissa the three frames share.
void trajectory_preview_window::render_scrub(const preview_run &shown)
{
    const int last = static_cast<int>(shown.samples.size()) - 1;
    if(last < 1)
        return;

    m_scrub                  = std::clamp(m_scrub, 0, last);
    const bool moved         = ImGui::SliderInt("Sample", &m_scrub, 0, last);
    const preview_sample &at = shown.samples[static_cast<std::size_t>(m_scrub)];

    ImGui::Text("%.2f s of %.2f s", at.at, shown.span);

    if(!moved)
        return;

    command(m_arm, [held = at.motion.position](robot_controller &control, scene_robot &) { control.preview_joint_configuration(held); });
}

// The two drawings are independent: a preview that goes away takes the commanded polyline with it
// and leaves the traversed one standing, which is what the comparison between the two is read from.
void trajectory_preview_window::draw_paths()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();
    if(published == nullptr)
        return;

    if(published->preview.get() != m_told_preview.get())
    {
        m_told_preview = published->preview;
        if(m_told_preview == nullptr)
            m_drawn.clear_pose_path(commanded_motion_path);
        else
            static_cast<void>(m_drawn.set_pose_path(commanded_motion_path, tool_poses_of(*m_told_preview)));
    }

    if(published->traversed.get() != m_told_traversed.get())
    {
        m_told_traversed = published->traversed;
        if(m_told_traversed != nullptr && m_told_traversed->size() >= least_drawn_poses)
            static_cast<void>(m_drawn.set_pose_path(traversed_motion_path, *m_told_traversed, threepp::Color(traversed_motion_tone)));
    }
}

}
