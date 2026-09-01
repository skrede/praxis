#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_TRAJECTORY_EXECUTOR_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_TRAJECTORY_EXECUTOR_H

#include "praxis/manipulator/scene_robot.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/scheduler/task.h"

#include "praxis/trajectory/trajectory.h"

#include <memory>
#include <filesystem>

namespace praxis::manipulator {

class motion_recording;

// The step a playback is registered at. A tick integrates the step it is handed, so the two policies
// reach the same end time and this is the density of the recorded motion rather than its speed.
inline constexpr scheduler::step_period playback_period{scheduler::seconds{0.004}};

// One motion and the recording it makes, advanced one step at a time by whoever services the strand
// it plays on.
class trajectory_executor
{
public:
    explicit trajectory_executor(scene_robot &driven);
    ~trajectory_executor();

    recording_parameters recording() const;
    void set_recording(recording_parameters parameters);

    // Loaded and not yet stepped, which is what says a playback still has to be registered.
    bool pending() const;
    bool executing() const;

    // Answers the folder the concluded recording was written into, and an empty path where there was
    // no recording to conclude.
    expected<std::filesystem::path, refusal> stop();

    bool execute(std::unique_ptr<trajectory::trajectory_generator> motion);

    // The factor scales the step into the trajectory's own time; the step itself is what the
    // recording is stamped with. A refused sample ends the motion where it was refused.
    expected<void, refusal> advance(scheduler::step_delta step, double factor);

private:
    bool m_started;
    double m_played;
    scene_robot &m_robot;
    scheduler::seconds m_elapsed;
    recording_parameters m_parameters;
    std::unique_ptr<motion_recording> m_recording;
    std::unique_ptr<trajectory::trajectory_generator> m_motion;

    expected<void, refusal> conclude();
};

}

#endif
