#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_COMPOSE_ARM_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_COMPOSE_ARM_H

#include "praxis/manipulator/slots.h"
#include "praxis/manipulator/types.h"
#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include <meios/model.h>

#include <memory>
#include <vector>
#include <functional>

namespace praxis::manipulator {

// What a window of one composed arm is built from. The stencil belongs to the preset and outlives
// every window built against it; the reader and the weak share are values a window keeps. The slots
// still holding an inert default are named as enumerators, so nothing a window keeps points into the
// composed aggregate. The chain is the one derived from this composition's own description, and the
// joint kinds are already implicit in it: a screw whose angular part is zero is a prismatic joint.
// The forward maps, the Jacobians and the path shapes are here and the solve is not, so a window has
// nothing to compose a holder from and no solve it could run on the render strand.
struct arm_window_inputs
{
    loadable_robot_stencil &stencil;
    arm_reader seen;
    std::weak_ptr<owned_arm> arm;
    rigid_motion::frame_ops frames;
    robot_slot_set inert;
    rigid_motion::screw_slot_set screw_inert;
    screw_chain chain;
    rigid_motion::screw_ops screw;
    forward_kinematics_ops fk;
    differential_kinematics_ops dk;
    trajectory::path_ops path;
};

// Invoked once per composition, after the publisher and the gated state exist and before the preset
// is constructed, which is the only point at which a window can be given both.
using arm_window_composer = std::function<std::vector<std::shared_ptr<scene::imgui_window>>(const arm_window_inputs &)>;

// One composition of an arm: the windows it opens, and which of the two models the arm can carry
// beside itself it draws. A model nothing composed a control over is a model nothing in the running
// application can hide, so the function deciding the windows is the function declaring the models
// and the two are carried together rather than paired by whoever calls them.
struct arm_composition
{
    arm_window_composer windows;
    bool draws_tool  = false;
    bool draws_world = false;
};

std::shared_ptr<scene::preset> compose_arm(const meios::model<> &description, const scene::preset_site &site, attached_models attached, const capabilities &arm,
                                           const trajectory::capabilities &shapes, const rigid_motion::capabilities &motions, const joint_vector &initial,
                                           const arm_window_composer &windows);

}

#endif
