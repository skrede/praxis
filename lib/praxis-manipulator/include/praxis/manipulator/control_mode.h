#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_CONTROL_MODE_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_CONTROL_MODE_H

#include <array>
#include <cstdint>

namespace praxis::manipulator {

// Contiguous from zero, so a combo box indexes the label table with the enumerator's own value.
enum class control_mode : std::uint8_t
{
    preview,
    simulation
};

const std::array<const char *, 2> &control_mode_labels();

}

#endif
