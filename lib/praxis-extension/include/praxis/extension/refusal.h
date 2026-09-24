#ifndef HPP_GUARD_PRAXIS_EXTENSION_REFUSAL_H
#define HPP_GUARD_PRAXIS_EXTENSION_REFUSAL_H

#include <cstdint>

namespace praxis {

// unsupported_input: this binding does not serve this shape, size or kind. degenerate: the input is
// ill-formed for the mathematics. no_solution: the input is well formed and no answer exists or was
// found. not_implemented: this binding does not implement this case.
enum class refusal : std::uint8_t
{
    unsupported_input,
    degenerate,
    no_solution,
    not_implemented
};

// The second axis a refusal is read on, beside its kind. per_request: a fact about the request that
// earned it, and another request may still be answered. composition_wide: a fact about the
// composition that answered -- true at every configuration of it, or across a whole region of them
// -- so no request of that shape will be answered either.
enum class refusal_standing : std::uint8_t
{
    per_request,
    composition_wide
};

// The third axis a refusal is read on: where the value the refused request was formed from came
// from. composed: a value the composition itself produced, so a request its mathematics cannot take
// says the composition cannot serve its own state. edited: a value an operator is moving, re-issued
// as it moves, so an ill-formed one says what was asked for and nothing about what answered.
enum class request_origin : std::uint8_t
{
    composed,
    edited
};

}

#endif
