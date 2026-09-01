#include "joint_naming.h"

#include "praxis/manipulator/joint_curve_window.h"
#include "praxis/manipulator/trajectory_configuration.h"

#include "praxis/rigid_motion/angles.h"

#include <imgui.h>

#include <Eigen/Core>

#include <string>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// The ordinate each frame is drawn against and the abscissa all three share. The joint quantities
// are in degrees, which is the unit every joint a learner types stands in.
constexpr const char *time_axis        = "Time (s)";
constexpr const char *position_axis    = "theta (deg)";
constexpr const char *rate_axis        = "dtheta/dt (deg/s)";
constexpr const char *rate_change_axis = "d2theta/dt2 (deg/s2)";

constexpr const char *unpublished_arm   = "The arm has published nothing yet.";
constexpr const char *nothing_previewed = "Ask for a preview to see how each joint moves through it.";
constexpr const char *nothing_chosen    = "Choose a joint to draw.";

using joint_quantity = trajectory::configuration trajectory::trajectory_sample::*;

bool drawn(const std::vector<std::size_t> &hidden, std::size_t joint)
{
    return std::find(hidden.begin(), hidden.end(), joint) == hidden.end();
}

void draw(std::vector<std::size_t> &hidden, std::size_t joint, bool shown)
{
    const auto found = std::find(hidden.begin(), hidden.end(), joint);
    if(shown && found != hidden.end())
        hidden.erase(found);
    else if(!shown && found == hidden.end())
        hidden.insert(std::upper_bound(hidden.begin(), hidden.end(), joint), joint);
}

// The curve is sampled at the samples' own times, so the abscissa is read off the samples rather
// than carried a second time beside them. A sample narrower than the joint asked for carries no
// value to draw, and the samples around it are still drawn.
scene::plot_series curve_of(std::string name, const preview_run &shown, Eigen::Index joint, joint_quantity value)
{
    scene::plot_series curve{std::move(name), {}, {}};
    curve.abscissa.reserve(shown.samples.size());
    curve.ordinate.reserve(shown.samples.size());
    for(const preview_sample &one : shown.samples)
        if(joint < (one.motion.*value).size())
        {
            curve.abscissa.push_back(one.at);
            curve.ordinate.push_back(to_degrees((one.motion.*value)[joint]));
        }

    return curve;
}

scene::plot_frame frame_of(const char *ordinate, const preview_run &shown, const std::vector<std::string> &named, const std::vector<std::size_t> &hidden, joint_quantity value)
{
    scene::plot_frame frame{.ordinate_label = ordinate, .logarithmic_ordinate = false, .series = std::vector<scene::plot_series>()};
    frame.series.reserve(named.size());
    for(std::size_t joint = 0u; joint < named.size(); ++joint)
        if(drawn(hidden, joint))
            frame.series.push_back(curve_of(named[joint], shown, static_cast<Eigen::Index>(joint), value));

    return frame;
}

scene::plot_reading plotted_curves(const preview_run *shown, const std::vector<std::size_t> &hidden)
{
    scene::plot_reading plotted{.message = std::string(), .abscissa_label = time_axis, .frames = std::vector<scene::plot_frame>()};
    if(shown == nullptr || shown->samples.empty())
    {
        plotted.message = nothing_previewed;

        return plotted;
    }

    const std::vector<std::string> named = named_joints(static_cast<std::size_t>(shown->samples.front().motion.position.size()));
    plotted.frames.push_back(frame_of(position_axis, *shown, named, hidden, &trajectory::trajectory_sample::position));
    plotted.frames.push_back(frame_of(rate_axis, *shown, named, hidden, &trajectory::trajectory_sample::velocity));
    plotted.frames.push_back(frame_of(rate_change_axis, *shown, named, hidden, &trajectory::trajectory_sample::acceleration));

    if(plotted.frames.front().series.empty())
    {
        plotted.frames.clear();
        plotted.message = nothing_chosen;
    }

    return plotted;
}

}

joint_curve_window::settings::settings(std::vector<std::size_t> left_out)
        : hidden(std::move(left_out))
{
}

joint_curve_window::joint_curve_window(std::string name, arm_reader seen)
        : joint_curve_window(std::move(name), std::move(seen), settings{})
{
}

joint_curve_window::joint_curve_window(std::string name, arm_reader seen, const settings &state, std::string at)
        : plot_window(
                  std::move(name), [this] { render_controls(); }, [this] { return reading(); })
        , m_seen(std::move(seen))
        , m_settings_at(std::move(at))
        , m_hidden(state.hidden)
{
    std::sort(m_hidden.begin(), m_hidden.end());
}

joint_curve_window::settings joint_curve_window::state() const
{
    return settings{m_hidden};
}

scene::plot_reading joint_curve_window::reading() const
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();

    return plotted_curves(published == nullptr ? nullptr : published->preview.get(), m_hidden);
}

std::vector<config::edit> joint_curve_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_joint_curves(state(), m_settings_at));
}

// One switch per joint the publication carries, so an arm of a different width offers a different
// number of them and a joint nothing published names is not offered at all.
void joint_curve_window::render_controls()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();
    if(published == nullptr)
    {
        ImGui::TextUnformatted(unpublished_arm);

        return;
    }

    const std::vector<std::string> named = named_joints(static_cast<std::size_t>(published->joints.size()));
    for(std::size_t joint = 0u; joint < named.size(); ++joint)
    {
        if(joint != 0u)
            ImGui::SameLine();

        bool shown = drawn(m_hidden, joint);
        if(ImGui::Checkbox(named[joint].c_str(), &shown))
            draw(m_hidden, joint, shown);
    }
}

}
