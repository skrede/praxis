#include "praxis/manipulator/arm_state.h"

#include "trajectory_executor.h"

#include "robot/manipulability.h"

#include "praxis/extension/held_handle.h"

#include <span>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::manipulator {

namespace {

expected<Eigen::Vector3d, refusal> position_in(const scene_robot &arm, const expected<transform, refusal> &pose)
{
    if(!pose)
        return unexpected(pose.error());

    return arm.position_of(*pose);
}

expected<rotation, refusal> orientation_in(const scene_robot &arm, const expected<transform, refusal> &pose)
{
    if(!pose)
        return unexpected(pose.error());

    return arm.orientation_of(*pose);
}

std::vector<std::vector<iteration_state>> solves_of(const robot_controller &control)
{
    const std::span<const std::vector<iteration_state>> recorded = control.solves();

    return std::vector<std::vector<iteration_state>>(recorded.begin(), recorded.end());
}

std::vector<joint_vector> solutions_of(const robot_controller &control)
{
    const std::span<const joint_vector> answered = control.solutions();

    return std::vector<joint_vector>(answered.begin(), answered.end());
}

std::vector<std::size_t> reached_of(const robot_controller &control)
{
    const std::span<const std::size_t> named = control.reached();

    return std::vector<std::size_t>(named.begin(), named.end());
}

bool ill_formed(const expected<manipulability_ellipsoid, refusal> &of)
{
    return !of && of.error() == refusal::degenerate;
}

// A Jacobian that is itself a refusal is answered for where it is taken, and a chain too narrow to
// carry an ellipsoid at any configuration says so in what it publishes rather than in the log. What
// is left is a Jacobian that was answered and whose own values no ellipsoid can be built from.
bool undecomposed(const expected<jacobian, refusal> &taken, const jacobian_manipulability &of)
{
    return taken.has_value() && (ill_formed(of.angular) || ill_formed(of.linear));
}

}

arm_state::arm_state(scheduler::strand own, std::shared_ptr<scene_robot> driven, std::shared_ptr<robot_controller> control, std::shared_ptr<arm_publisher> published)
        : m_refused(false)
        , m_undecomposed(false)
        , m_own(own)
        , m_motion()
        , m_last_motion()
        , m_robot(std::move(driven))
        , m_published(std::move(published))
        , m_controller(std::move(control))
{
    held(m_robot, "the arm state", "driven robot");
    held(m_published, "the arm state", "publication");
    held(m_controller, "the arm state", "controller");
    publish();
}

// The handler is registered on the strand rather than through the gate, which is where it already
// runs, so a tick costs one service instead of two.
void arm_state::service_playback()
{
    if(!m_controller->playback_pending() || m_motion.active())
        return;

    m_motion = m_own.every(playback_period, scheduler::overrun::catch_up, [this](scheduler::step_delta step) { tick(step); });
}

// A cancel empties the receipt, so the tallies are taken while it still has them: they are the only
// thing left of a motion once its task is gone. A service that replays past the end of a motion
// finds nothing to advance, which is what keeps those tallies from being overwritten with none.
void arm_state::tick(scheduler::step_delta step)
{
    if(!m_controller->executing())
        return;

    // The conclusion of one motion is what the next target of a queued run waits on, so the run is
    // carried on here rather than from a poll of its own.
    advance(
            [step](robot_controller &control, scene_robot &)
            {
                if(!control.advance_playback(step))
                    control.start_queued();
            });
    if(m_controller->executing())
        return;

    m_last_motion = m_motion.counters();
    m_motion.cancel();
    publish();
}

void arm_state::publish()
{
    arm_snapshot assembled = assemble();
    report_pose_refusal(assembled);
    report_manipulability_refusal(assembled);
    m_published->publish(std::make_shared<const arm_snapshot>(std::move(assembled)));
}

arm_snapshot arm_state::assemble() const
{
    const joint_vector at                     = m_robot->joint_positions();
    const expected<transform, refusal> flange = m_robot->flange_pose();
    const expected<transform, refusal> tool   = m_robot->tool_pose();
    const expected<jacobian, refusal> space   = m_robot->solver().space_jacobian(at);
    const expected<jacobian, refusal> body    = m_robot->solver().body_jacobian(at);

    return arm_snapshot{at,
                        m_robot->limits(),
                        m_robot->tool_offset(),
                        tool,
                        flange,
                        position_in(*m_robot, tool),
                        position_in(*m_robot, flange),
                        orientation_in(*m_robot, tool),
                        orientation_in(*m_robot, flange),
                        m_controller->recording(),
                        m_controller->velocity_factor(),
                        m_controller->executing(),
                        m_last_motion,
                        solves_of(*m_controller),
                        space,
                        body,
                        manipulability_of(space),
                        manipulability_of(body),
                        solutions_of(*m_controller),
                        reached_of(*m_controller),
                        m_controller->preview(),
                        m_controller->traversed(),
                        m_controller->time_scaling(),
                        m_controller->trapezoid_bounds()};
}

void arm_state::report_pose_refusal(const arm_snapshot &assembled)
{
    if(assembled.tool_pose && assembled.flange_pose)
    {
        m_refused = false;
        return;
    }

    if(!std::exchange(m_refused, true))
        m_controller->report_refusal("fk.forward_kinematics", assembled.flange_pose ? assembled.tool_pose.error() : assembled.flange_pose.error());
}

// A latch of its own, so a standing decomposition refusal and the pose refusal beside it never
// silence each other. The kind is the one the block that was answered produced, and its standing is
// composition-wide: a composition that cannot be decomposed answers everything else it was composed
// for, so nothing is asked to unload.
void arm_state::report_manipulability_refusal(const arm_snapshot &assembled)
{
    if(!undecomposed(assembled.space_jacobian, assembled.space_manipulability) && !undecomposed(assembled.body_jacobian, assembled.body_manipulability))
    {
        m_undecomposed = false;
        return;
    }

    if(!std::exchange(m_undecomposed, true))
        m_controller->report_refusal("dk.manipulability", refusal::degenerate, refusal_standing::composition_wide);
}

}
