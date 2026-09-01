#include "trajectory_preview_frames.h"

#include "praxis/manipulator/control_parameters_window.h"

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::manipulator {

namespace {

constexpr const char *nothing_previewed = "Ask for a preview to see the motion it would play.";
constexpr const char *nothing_travelled = "This motion stands where it began, so there is no timing to draw.";
constexpr const char *nothing_chosen    = "Choose a curve to draw.";

// What the one curve is named where the motion answers for no choice, and so carries no marker
// saying which choice it is: no choice governs it.
constexpr const char *own_timing = "Its own timing";

std::string named(std::size_t which, time_scaling_choice chosen)
{
    const std::string label = which < time_scaling_labels.size() ? time_scaling_labels[which] : std::to_string(which);

    return which == static_cast<std::size_t>(chosen) ? label + " (chosen)" : label;
}

// The curves are sampled at the samples' own times, so the abscissa is read off the samples rather
// than carried a second time beside them.
scene::plot_series curve_of(std::string name, std::span<const trajectory::scaling_sample> sampled, const preview_run &shown, double trajectory::scaling_sample::*value)
{
    scene::plot_series drawn{std::move(name), {}, {}};
    drawn.abscissa.reserve(sampled.size());
    drawn.ordinate.reserve(sampled.size());
    for(std::size_t at = 0; at < sampled.size() && at < shown.samples.size(); ++at)
    {
        drawn.abscissa.push_back(shown.samples[at].at);
        drawn.ordinate.push_back(sampled[at].*value);
    }

    return drawn;
}

bool stands_beside(const preview_run &shown)
{
    for(const std::vector<trajectory::scaling_sample> &candidate : shown.scaling)
        if(!candidate.empty())
            return true;

    return false;
}

scene::plot_frame frame_of(const char *ordinate, const preview_run &shown, time_scaling_choice chosen, double trajectory::scaling_sample::*value)
{
    scene::plot_frame frame{.ordinate_label = ordinate, .logarithmic_ordinate = false, .series = std::vector<scene::plot_series>()};
    if(!stands_beside(shown))
    {
        frame.series.push_back(curve_of(own_timing, shown.parameter, shown, value));

        return frame;
    }

    frame.series.reserve(shown.scaling.size());
    for(std::size_t which = 0; which < shown.scaling.size(); ++which)
        frame.series.push_back(curve_of(named(which, chosen), shown.scaling[which], shown, value));

    return frame;
}

}

scene::plot_reading plotted_preview(const preview_run *shown, time_scaling_choice chosen, const shown_frames &drawn)
{
    scene::plot_reading plotted{.message = std::string(), .abscissa_label = time_axis, .frames = std::vector<scene::plot_frame>()};
    if(shown == nullptr)
    {
        plotted.message = nothing_previewed;

        return plotted;
    }
    if(!stands_beside(*shown) && shown->parameter.empty())
    {
        plotted.message = nothing_travelled;

        return plotted;
    }

    if(drawn.parameter)
        plotted.frames.push_back(frame_of(parameter_axis, *shown, chosen, &trajectory::scaling_sample::s));
    if(drawn.rate)
        plotted.frames.push_back(frame_of(rate_axis, *shown, chosen, &trajectory::scaling_sample::ds));
    if(drawn.rate_change)
        plotted.frames.push_back(frame_of(rate_change_axis, *shown, chosen, &trajectory::scaling_sample::dds));

    if(plotted.frames.empty())
        plotted.message = nothing_chosen;

    return plotted;
}

}
