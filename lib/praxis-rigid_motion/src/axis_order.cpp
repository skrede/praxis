#include "praxis/rigid_motion/axis_order.h"

#include <cstddef>

namespace praxis {

namespace {

constexpr std::array<const char *, 12> labels{"XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX", "XYX", "XZX", "YXY", "YZY", "ZXZ", "ZYZ"};

constexpr std::uint8_t index_of(char axis)
{
    if(axis == 'X')
        return std::uint8_t{0};
    if(axis == 'Y')
        return std::uint8_t{1};
    return std::uint8_t{2};
}

}

std::span<const char *const> axis_order_labels()
{
    return std::span<const char *const>(labels);
}

std::array<std::uint8_t, 3> axis_indices(axis_order order)
{
    const char *label = labels[static_cast<std::size_t>(order)];
    return {index_of(label[0]), index_of(label[1]), index_of(label[2])};
}

}
