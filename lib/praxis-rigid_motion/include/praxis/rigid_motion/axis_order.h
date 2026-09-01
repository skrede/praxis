#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_AXIS_ORDER_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_AXIS_ORDER_H

#include <span>
#include <array>
#include <cstdint>

namespace praxis {

// Contiguous from zero, so a combo box indexes the label table with the enumerator's own value.
enum class axis_order : std::uint8_t
{
    xyz,
    xzy,
    yxz,
    yzx,
    zxy,
    zyx,
    xyx,
    xzx,
    yxy,
    yzy,
    zxz,
    zyz
};

std::span<const char *const> axis_order_labels();

std::array<std::uint8_t, 3> axis_indices(axis_order order);

}

#endif
