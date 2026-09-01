#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_TRAJECTORY_PREVIEW_FRAMES_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_TRAJECTORY_PREVIEW_FRAMES_H

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/motion_commands.h"

#include "praxis/scene/plot_window.h"

namespace praxis::manipulator {

// The ordinate each frame is drawn against, and the abscissa all three share. The rates are per
// second and per second squared over the path parameter, which is itself dimensionless.
inline constexpr const char *time_axis        = "Time (s)";
inline constexpr const char *parameter_axis   = "s";
inline constexpr const char *rate_axis        = "ds/dt (1/s)";
inline constexpr const char *rate_change_axis = "d2s/dt2 (1/s2)";

// Which of the three a window is showing.
struct shown_frames
{
    bool parameter;
    bool rate;
    bool rate_change;
};

// One frame per shown ordinate. Where the run holds a scaling the motion could have been commanded
// under, each frame carries one curve per scaling, in the choice enumeration's own order and named
// so the one the arm is set to is identifiable. Where it holds none, each frame carries the one
// curve the motion has -- the path parameter it realizes -- named for the motion and marked for no
// choice. A run holding neither answers a message instead, as an absent run does. No frame is drawn
// on a logarithmic ordinate: the second derivative goes negative.
scene::plot_reading plotted_preview(const preview_run *shown, time_scaling_choice chosen, const shown_frames &drawn);

}

#endif
