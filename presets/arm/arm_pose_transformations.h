#ifndef HPP_GUARD_PRAXIS_PRESETS_ARM_ARM_POSE_TRANSFORMATIONS_H
#define HPP_GUARD_PRAXIS_PRESETS_ARM_ARM_POSE_TRANSFORMATIONS_H

#include "praxis/manipulator/robot.h"
#include "praxis/manipulator/slots.h"

#include <array>
#include <string>

namespace praxis::presets {

// The four frame transformations every pose a scenario shows or commands is read through. Each of them
// answers the origin unrotated where nobody bound it, and the origin unrotated is a pose an arm can
// genuinely be at, so downstream of them a fabrication and a reading are the same value.
constexpr std::array<manipulator::robot_slot, 4> pose_transformations{manipulator::robot_slot::tool_pose_from_flange_pose, manipulator::robot_slot::flange_pose_from_tool_pose,
                                                                      manipulator::robot_slot::position_from_pose, manipulator::robot_slot::orientation_from_pose};

// Which of the four a composition left at their defaults, under the names the descriptor table
// carries them by, comma-separated and in the enumeration's order. Empty where all four are bound.
inline std::string unbound_pose_transformations(const manipulator::robot_slot_set &defaulted)
{
    constexpr manipulator::robot_ops slot_names{};

    return joined_slot_names(slot_names, defaulted_among(defaulted, pose_transformations));
}

}

#endif
