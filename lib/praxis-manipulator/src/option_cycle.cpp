#include "praxis/manipulator/option_cycle.h"

#include <cstddef>

namespace praxis::manipulator::detail {

std::optional<std::uint8_t> index_of_label(std::span<const char *const> labels, std::string_view label)
{
    for(std::size_t i = 0; i < labels.size(); ++i)
        if(label == labels[i])
            return static_cast<std::uint8_t>(i);

    return std::nullopt;
}

}
