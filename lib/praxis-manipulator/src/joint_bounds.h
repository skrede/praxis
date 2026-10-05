#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_JOINT_BOUNDS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_JOINT_BOUNDS_H

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"

#include <optional>

namespace praxis::manipulator {

// A joint that turns whole stands in the same place under every value a whole turn from the one naming
// it, so its bounds admit whichever of those namings falls between them; any other joint is admitted
// only at its own value. A bound pair the chain does not carry for a joint leaves that joint free.
std::optional<joint_vector> named_inside_bounds(const screw_chain &chain, const joint_vector &candidate);

// The projection onto the bounds of Kanzow, Yamashita & Fukushima, J. Comput. Appl. Math. 172, 2004: a
// joint takes its naming inside them when one exists, else, turning whole, the bound nearer on the
// circle (a tie to the bound crossed), else the bound crossed. A joint without a bound pair is free.
joint_vector projected_inside_bounds(const screw_chain &chain, const joint_vector &raw);

}

#endif
