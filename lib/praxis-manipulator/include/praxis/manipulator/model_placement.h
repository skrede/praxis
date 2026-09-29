#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_MODEL_PLACEMENT_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_MODEL_PLACEMENT_H

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/world_object_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/slots.h"

#include <memory>

namespace praxis::manipulator {

// The frame slots a placement read that still hold their defaults, and those that answered a value
// that is not finite from arguments that were. A placement that reads nothing leaves both empty.
struct placement_doubts
{
    rigid_motion::frame_slot_set defaulted;
    rigid_motion::frame_slot_set not_finite;
};

// An active tool the stencil holds at the flange is drawn at the scale and graphics transform
// `state` gives it, and the arm is posted its kinematics offset. Any other tool is taken off the
// flange and the arm's tool offset returns to the identity. Either way the placement is what the
// slots answered.
placement_doubts seat_tool(loadable_robot_stencil &on, const std::weak_ptr<owned_arm> &arm, const rigid_motion::frame_ops &frames, const tool_window::settings &state);

// An active world object the stencil holds is drawn at the scale, position and rotation `state` gives
// it, and an inactive one is taken out of the stencil. A stencil holding none is left as it is.
placement_doubts place_world_object(loadable_robot_stencil &on, const rigid_motion::frame_ops &frames, const world_object_window::settings &state);

}

#endif
