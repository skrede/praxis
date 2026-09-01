#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ARM_STATE_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ARM_STATE_H

#include "praxis/manipulator/scene_robot.h"
#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/scheduler/task.h"
#include "praxis/scheduler/strand.h"
#include "praxis/scheduler/ownership.h"

#include <memory>
#include <utility>

namespace praxis::manipulator {

// The state one preset serializes on its own strand. Construction publishes, and so does every
// mutation, so a reader taken afterwards never reads an absent value and a readout moves whether or
// not a motion is playing. A mutation that leaves a motion waiting registers the playback before it
// publishes, so the publication that reports the motion already has a task behind it.
class arm_state
{
public:
    arm_state(scheduler::strand own, std::shared_ptr<scene_robot> driven, std::shared_ptr<robot_controller> control, std::shared_ptr<arm_publisher> published);

    template<typename operation>
    void mutate(operation work)
    {
        const robot_controller::command_extent extent(*m_controller);
        advance(std::move(work));
    }

private:
    // The controller names the robot by reference, so it is declared after it and destroyed before it.
    bool m_refused;
    bool m_undecomposed;
    scheduler::strand m_own;
    scheduler::task_handle m_motion;
    scheduler::task_counters m_last_motion;
    std::shared_ptr<scene_robot> m_robot;
    std::shared_ptr<arm_publisher> m_published;
    std::shared_ptr<robot_controller> m_controller;

    // A playback advance is the same command still running, so it begins none. Both paths service a
    // waiting playback before publishing, which is what keeps a publication reporting a motion from
    // arriving before the task behind it.
    template<typename operation>
    void advance(operation work)
    {
        work(*m_controller, *m_robot);
        service_playback();
        publish();
    }

    void publish();

    void service_playback();

    void tick(scheduler::step_delta step);

    arm_snapshot assemble() const;

    void report_pose_refusal(const arm_snapshot &assembled);

    void report_manipulability_refusal(const arm_snapshot &assembled);
};

using owned_arm = scheduler::strand_owned<arm_state>;

// An expired observer means the preset is gone, so the command is not sent and nothing is reported.
// The admission verdict is discarded because between an unload being asked for and a window leaving
// the list there is at least one frame in which the strand legitimately admits nothing.
template<typename operation>
void command(const std::weak_ptr<owned_arm> &arm, operation work)
{
    const std::shared_ptr<owned_arm> gated = arm.lock();
    if(gated == nullptr)
        return;

    static_cast<void>(gated->with([work = std::move(work)](arm_state &state) mutable { state.mutate(std::move(work)); }));
}

}

#endif
