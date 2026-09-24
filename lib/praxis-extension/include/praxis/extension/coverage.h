#ifndef HPP_GUARD_PRAXIS_EXTENSION_COVERAGE_H
#define HPP_GUARD_PRAXIS_EXTENSION_COVERAGE_H

#include "praxis/extension/refusal.h"
#include "praxis/extension/slot_set.h"
#include "praxis/extension/descriptor.h"

#include "praxis/compat/expected.h"

#include <span>
#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <concepts>
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

// A capability an enumeration indexes and a view can be taken over. The accessor is found in the
// capability's own namespace at the point of use, which is where its header is visible.
template<typename Ops>
concept described_capability = requires(const Ops &ops) {
    typename capability_slots_t<Ops>;
    { view_of(ops) } -> std::same_as<capability_view>;
};

// Which of the slots a caller needs still hold their defaults. The answer is a set, so the caller
// decides on the set: an empty one is the only thing that says every needed slot is bound, and no
// rendering of a name stands between the question and the answer.
template<slot_enumeration Slot, std::size_t N>
constexpr basic_slot_set<Slot> defaulted_among(const basic_slot_set<Slot> &defaulted, const std::array<Slot, N> &needed)
{
    basic_slot_set<Slot> wanted;
    for(const Slot slot : needed)
    {
        wanted.set(slot);
    }

    return defaulted & wanted;
}

// The names a set's slots carry in the descriptor table of the capability whose slots they are, in
// enumeration order and comma-separated, for a message. The constraint is the relation: a set whose
// enumeration indexes another capability's table is not a set this names.
template<typename Ops, slot_enumeration Slot>
    requires described_capability<Ops> && std::same_as<Slot, capability_slots_t<Ops>>
std::string joined_slot_names(const Ops &described, const basic_slot_set<Slot> &listed)
{
    const capability_view described_slots = view_of(described);

    std::string names;
    for(std::size_t index = 0; index < static_cast<std::size_t>(Slot::count); ++index)
    {
        if(listed.contains(static_cast<Slot>(index)))
        {
            if(!names.empty())
            {
                names += ", ";
            }
            names += slot_name(described_slots, index);
        }
    }

    return names;
}

}

#endif
