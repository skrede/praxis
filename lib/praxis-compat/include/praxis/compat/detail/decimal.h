#ifndef HPP_GUARD_PRAXIS_COMPAT_DETAIL_DECIMAL_H
#define HPP_GUARD_PRAXIS_COMPAT_DETAIL_DECIMAL_H

#include <optional>
#include <string_view>

namespace praxis::detail {

// The double the whole of `text` spells in the C locale's decimal grammar, whatever the process's
// locale; nothing where it spells none, or one that underflows to zero or overflows.
std::optional<double> decimal_value(std::string_view text);

}

#endif
