#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_CAPABILITIES_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_CAPABILITIES_H

#include "praxis/manipulator/slots.h"

#include "praxis/extension/descriptor.h"

#include <array>

namespace praxis::manipulator {

struct capabilities
{
    forward_kinematics_ops fk{};
    differential_kinematics_ops dk{};
    inverse_kinematics_ops ik{};
    robot_ops robot{};
    motion_ops motion{};
    modeling_ops modeling{};
    task_trajectory_ops trajectory{};
};

capabilities baseline();

// The views are in the aggregate's member order and point into the value passed, which must outlive
// them. A temporary argument would leave every view dangling at the end of the full expression, so
// that call is deleted rather than diagnosed at run time.
std::array<capability_view, 7> capability_views(const capabilities &c);
std::array<capability_view, 7> capability_views(capabilities &&) = delete;

}

#endif
