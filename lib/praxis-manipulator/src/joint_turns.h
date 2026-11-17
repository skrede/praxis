#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_JOINT_TURNS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_JOINT_TURNS_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/screw_chain.h"

#include <optional>

namespace praxis::manipulator {

// Lynch & Park, Modern Robotics, Def. 3.24: a unit screw of pitch w . v = 0 is a pure rotation, so a
// whole turn about it is no displacement. Any other screw answers zero.
double whole_turn(const screw_axis &s);

// The configuration standing where q does, each joint that turns whole taken at its equivalent
// nearest the same joint of near, within the chain's position limits. None when a joint has no such
// value, or when q or near is not one entry per joint.
std::optional<joint_vector> turned_within_limits(const screw_chain &chain, const joint_vector &q, const joint_vector &near);

}

#endif
