#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_DESCRIBED_SLOTS_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_DESCRIBED_SLOTS_H

#include "praxis/manipulator/slots.h"

#include <cstddef>

// Counted from the slot enumerations rather than written out, so a slot added to a capability moves
// every suite reading these and none of them carries a number of its own to forget.
namespace praxis::fixture {

template<typename Slot>
inline constexpr std::size_t slots_of = static_cast<std::size_t>(Slot::count);

inline constexpr std::size_t described_slots = slots_of<manipulator::forward_kinematics_slot> + slots_of<manipulator::differential_kinematics_slot> +
        slots_of<manipulator::inverse_kinematics_slot> + slots_of<manipulator::robot_slot> + slots_of<manipulator::motion_slot> + slots_of<manipulator::modeling_slot> +
        slots_of<manipulator::task_trajectory_slot>;

// Every described slot has a row in the tables.
inline constexpr std::size_t compared_slots = described_slots;

}

#endif
