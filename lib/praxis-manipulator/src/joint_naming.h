#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_JOINT_NAMING_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_JOINT_NAMING_H

#include <string>
#include <vector>
#include <cstddef>

namespace praxis::manipulator {

// What a joint is called wherever one is drawn beside the others, counted from one so that the
// first joint reads as the first rather than as the zeroth.
std::vector<std::string> named_joints(std::size_t joints);

}

#endif
