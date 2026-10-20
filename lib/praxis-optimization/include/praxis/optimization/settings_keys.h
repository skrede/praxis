#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SETTINGS_KEYS_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SETTINGS_KEYS_H

#include <array>

namespace praxis::optimization {

// The keys a scenario's windows write its settings under, at the root of its document, and that the
// document is declared and read by. A table of two is indexed by the variable, x then y.
inline constexpr const char *function_name_key = "function/name";
inline constexpr const char *limit_key         = "solve/limit";
inline constexpr const char *step_key          = "solve/step";

inline constexpr std::array<const char *, 2> range_lower_keys{"range/x_lower", "range/y_lower"};
inline constexpr std::array<const char *, 2> range_upper_keys{"range/x_upper", "range/y_upper"};
inline constexpr std::array<const char *, 2> seed_keys{"seed/x", "seed/y"};

}

#endif
