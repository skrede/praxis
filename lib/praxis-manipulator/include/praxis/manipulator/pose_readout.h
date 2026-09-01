#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_POSE_READOUT_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_POSE_READOUT_H

#include "praxis/manipulator/slots.h"
#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/option_cycle.h"

#include "praxis/scene/labeled_value_window.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/axis_order.h"

#include <memory>
#include <string>
#include <cstdint>

namespace praxis::manipulator {

class pose_readout
{
public:
    enum class frame_view : std::uint8_t
    {
        flange,
        tool
    };

    // A reading the composition never bound and one a capability declined are different facts, and
    // the unbound answer is the earlier of the two: the accessors that would refuse are not reached.
    enum class pose_reading : std::uint8_t
    {
        value,
        unpublished,
        refused,
        position_unbound,
        orientation_unbound,
        both_unbound
    };

    pose_readout(arm_reader seen, const rigid_motion::frame_ops &injected, robot_slot_set inert);

    void render_controls();

    scene::readout reading() const;

    pose_reading reading_of(const arm_snapshot &seen, frame_view frame) const;

private:
    arm_reader m_seen;
    robot_slot_set m_inert;
    axis_order m_euler_order;
    rigid_motion::frame_ops m_frame;
    option_cycle<frame_view, 2> m_frame_view;
};

std::shared_ptr<scene::labeled_value_window> compose_pose_readout(std::string name, arm_reader seen, const rigid_motion::frame_ops &injected, robot_slot_set inert);

}

#endif
