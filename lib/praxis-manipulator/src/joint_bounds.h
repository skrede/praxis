#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_JOINT_BOUNDS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_JOINT_BOUNDS_H

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"

#include <optional>

namespace praxis::manipulator {

// Lynch & Park, Modern Robotics, Def. 3.24: a unit screw of pitch w . v = 0 is a pure rotation, so a
// whole turn about it is no displacement. Any other screw answers zero.
double whole_turn(const screw_axis &s);

// A joint that turns whole stands in the same place under every value a whole turn from the one naming
// it, so its bounds admit whichever of those namings falls between them; any other joint is admitted
// only at its own value. A bound pair the chain does not carry for a joint leaves that joint free.
std::optional<joint_vector> named_inside_bounds(const screw_chain &chain, const joint_vector &candidate);

}

#endif
