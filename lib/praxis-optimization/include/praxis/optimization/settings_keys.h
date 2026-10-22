#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SETTINGS_KEYS_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SETTINGS_KEYS_H

#include <array>

namespace praxis::optimization {

// The keys a scenario's windows write its settings under, at the root of its document, and that the
// document is declared and read by. A table of two is indexed by the variable, x then y.
inline constexpr const char *function_name_key = "function/name";
inline constexpr const char *source_key        = "function/source";
inline constexpr const char *expression_key    = "function/expression";
inline constexpr const char *limit_key         = "solve/limit";
inline constexpr const char *step_key          = "solve/step";
inline constexpr const char *form_key          = "solve/form";
inline constexpr const char *method_key        = "solve/method";
inline constexpr const char *step_length_key   = "solve/step_length";

// A typed function's parameters, one instance per letter, keyed by the letter.
inline constexpr const char *parameters_key       = "parameters/parameter";
inline constexpr const char *parameter_name_leaf  = "name";
inline constexpr const char *parameter_value_leaf = "value";
inline constexpr const char *parameter_lower_leaf = "lower";
inline constexpr const char *parameter_upper_leaf = "upper";

inline constexpr std::array<const char *, 2> range_lower_keys{"range/x_lower", "range/y_lower"};
inline constexpr std::array<const char *, 2> range_upper_keys{"range/x_upper", "range/y_upper"};
inline constexpr std::array<const char *, 2> seed_keys{"seed/x", "seed/y"};

// The spellings the form, the method and the source are written in, indexed by problem_form,
// minimizer_slot and function_source.
inline constexpr std::array<const char *, 2> form_spellings{"root", "minimum"};
inline constexpr std::array<const char *, 2> method_spellings{"newton", "gradient_descent"};
inline constexpr std::array<const char *, 2> source_spellings{"registered", "manual"};

}

#endif
