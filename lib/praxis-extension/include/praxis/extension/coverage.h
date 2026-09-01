#ifndef HPP_GUARD_PRAXIS_EXTENSION_COVERAGE_H
#define HPP_GUARD_PRAXIS_EXTENSION_COVERAGE_H

#include "praxis/extension/refusal.h"
#include "praxis/extension/descriptor.h"

#include "praxis/compat/expected.h"

#include <span>
#include <vector>
#include <cstddef>
#include <string_view>

namespace praxis {

// True when F is a pointer to a function whose return type carries the refusal channel.
template<typename F>
inline constexpr bool returns_refusal_v = false;

template<typename T, typename... Args>
inline constexpr bool returns_refusal_v<expected<T, refusal> (*)(Args...)> = true;

struct defaulted_slot
{
    std::string_view extension;
    std::string_view slot;
};

// Both index-taking functions are total: an index past the end of the view's descriptors is not a
// slot, and answers false and the empty name rather than reading past the span.
std::size_t count_defaults(std::span<const capability_view> views);
std::vector<defaulted_slot> defaulted_slots(std::span<const capability_view> views);
bool holds_default(const capability_view &view, std::size_t index);
std::string_view slot_name(const capability_view &view, std::size_t index);

}

#endif
