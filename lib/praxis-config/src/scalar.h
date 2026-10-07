#ifndef HPP_GUARD_PRAXIS_CONFIG_SCALAR_H
#define HPP_GUARD_PRAXIS_CONFIG_SCALAR_H

#include <cstdint>
#include <optional>
#include <string_view>

namespace praxis::config {

std::optional<bool> as_flag(std::string_view text);

std::optional<double> as_real(std::string_view text);

// Whether `text` spells NaN or an infinity in any form std::strtod reads.
bool non_finite(std::string_view text);

std::optional<std::int64_t> as_integer(std::string_view text);

}

#endif
